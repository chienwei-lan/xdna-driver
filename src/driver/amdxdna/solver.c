// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022-2024, Advanced Micro Devices, Inc.
 */

#include <linux/slab.h>
#include "solver.h"

static u32 calculate_gops(struct aie_qos *rqos)
{
	u32 service_rate = 0;

	if (rqos->latency)
		service_rate = (1000 / rqos->latency);

	if (rqos->fps > service_rate)
		return rqos->fps * rqos->gops;

	return service_rate * rqos->gops;
}

/*
 * qos_meet() - Check the QOS request can be met.
 *
 * @xrs:	Softstate of xrs
 * @rqos:	Requested QoS
 * @cgops:	giga ops Capability
 *
 * Return:	0 when successful or standard error number when failing
 *		Failing scenarios
 *			1. can't meet qos requirement
 */
static int qos_meet(struct solver_state *xrs, struct aie_qos *rqos, u32 cgops)
{
	u32 request_gops = calculate_gops(rqos) * xrs->cfg.sys_eff_factor;

	if (request_gops <= cgops)
		return 0;

	return -EINVAL;
}

/*
 * sanity_check() - Do a basic sanity check on allocation requests.
 *
 * @pmp:	Input partition metadata (for Fat-XCLBIN)
 * @rqos:	Requested QoS
 * @xrs:	Soft state of xrs
 *
 * Return:	0 when successful or standard error number when failing
 *		Failing scenarios
 *			1. Invalid rqos
 *			2. GOPs in requested QoS exceed all CDO groups
 *			   GOPs capabilities.
 */
static int sanity_check(struct part_meta *pmp, struct aie_qos *rqos, struct solver_state *xrs)
{
	struct cdo_parts *cdop = pmp->cdo;
	u32 cu_clk_freq;

	if (!rqos)
		return -EINVAL;

	/*
	 * We can find at least one CDOs groups that meet the
	 * GOPs requirement.
	 */
	cu_clk_freq = xrs->cfg.clk_list.cu_clk_list[xrs->cfg.clk_list.num_levels - 1];

	if (qos_meet(xrs, rqos, cdop->qos_cap->opc * cu_clk_freq / 1000))
		return -EINVAL;

	return 0;
}

static struct solver_node *rg_search_node(struct solver_rgroup *rgp, u64 rid)
{
	struct solver_node *node;

	list_for_each_entry(node, &rgp->node_list, list) {
		if (node->rid == rid)
			return node;
	}

	return NULL;
}

static void remove_partition_node(struct solver_rgroup *rgp,
				  struct partition_node *pt_node)
{
	pt_node->nshared--;
	if (pt_node->nshared > 0)
		return;

	list_del(&pt_node->list);
	rgp->npartition_node--;
	rgp->allocated -= pt_node->ncol;

	bitmap_clear(rgp->resbit, pt_node->start_col, pt_node->ncols);
	kfree(pt_node);
}

static void remove_solver_node(struct solver_rgroup *rgp,
			       struct solver_node *node)
{
	list_del(&node->list);
	rgp->nnode--;

	if (node->pt_node)
		remove_partition_node(rgp, node->pt_node);

	kfree(node);
}

static struct solver_node *allocate_solver_node(struct solver_state *xrs,
						struct alloc_requests *req)
{
	struct cdo_parts *cdop = req->pmp->cdo;
	struct solver_node *node;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return NULL;

	uuid_copy(&node->xclbin_uuid, req->pmp->xclbin_uuid);
	uuid_copy(&node->cdo_uuid, cdop->cdo_uuid);
	node->rid = req->rid;
	node->first_col = cdop->first_col;
	node->ncols = cdop->ncols;

	memcpy(&node->qos_cap, cdop->qos_cap, sizeof(struct aie_qos_cap));
	memcpy(&node->rqos, req->rqos, sizeof(struct aie_qos));

	list_add_tail(&node->list, &xrs->rgp.node_list);
	xrs->rgp.nnode++;
	return node;
}

static int get_free_partition(struct solver_state *xrs, u32 first_col, u32 ncols,
			      u32 total_col, u32 *start_col)
{
	u32 idx;

	for (idx = first_col; idx < total_col; idx++) {
		if (find_next_bit(xrs->rgp.resbit, XRS_MAX_COL, idx) >=
		    idx + ncols) {
			*start_col = idx;
			return 0;
		}
	}

	return -ENODEV;
}

static void fill_partition_node(struct partition_node *pt_node,
				u32 start_col, u32 ncols, struct aie_qos *rqos)
{
	pt_node->nshared = 1;
	pt_node->start_col = start_col;
	pt_node->ncols = ncols;

	/*
	 * Before fully support latency in QoS, if a request
	 * specifies a non-zero latency value, it will not share
	 * the partition with other requests.
	 */
	if (rqos->latency)
		pt_node->exclusive = true;

	memcpy(&pt_node->pqos, rqos, sizeof(struct aie_qos));
}

static void add_partition_node(struct solver_rgroup *rgp,
			       struct partition_node *pt_node)
{
	list_add_tail(&pt_node->list, &rgp->pt_node_list);

	rgp->npartition_node++;
	rgp->allocated += pt_node->ncol;
	bitmap_set(rgp->resbit, pt_node->start_col, pt_node->ncol);
}

static int allocate_partition(struct solver_state *xrs, struct part_meta *pmp,
			      struct aie_qos *rqos, struct solver_node *snode)
{
	struct partition_node *pt_node, *rpt_node = NULL;
	u32 total_col = xrs->cfg.total_col;
	struct cdo_parts *cdop = pmp->cdo;
	int j, ret;

	ret = get_free_partition(xrs, cdop->first_col, cdop->ncols, total_col,
				 &snode->start_col);
	if (!ret) {
		/* got free partition */
		pt_node = kzalloc(sizeof(*pt_node), GFP_KERNEL);
		if (!pt_node)
			return -ENOMEM;

		fill_partition_node(pt_node, snode->start_col,
				    snode->ncols, &snode->rqos);
		add_partition_node(&xrs->rgp, pt_node);
		snode->pt_node = pt_node;

		return 0;
	}

	if (xrs->cfg.mode != XRS_MODE_TEMPORAL_BEST) {
		dev_err(xrs->cfg.dev, "no available partition");
		return -ENODEV;
	}

	/* try to get a share-able partition */
	list_for_each_entry(pt_node, &xrs->rgp.pt_node_list, list) {
		if (pt_node->exclusive)
			continue;

		if (rpt_node && pt_node->nshared >= rpt_node->nshared)
			continue;

		for (j = cdop->first_col; j < total_col; j++) {
			if (j == pt_node->start_col &&
			    cdop->ncols == pt_node->ncols) {
				rpt_node = pt_node;
				snode->start_col = j;
				break;
			}
		}
	}

	if (!rpt_node)
		return -ENODEV;

	rpt_node->nshared++;
	snode->pt_node = rpt_node;

	return 0;
}

static void fill_load_action(struct solver_state *xrs,
			     struct solver_node *snode,
			     struct xrs_action_load *action)
{
	action->rid = snode->rid;
	action->xclbin_uuid = &snode->xclbin_uuid;
	action->cdo_uuid = &snode->cdo_uuid;
	action->part.start_col = snode->pt_node->start_col;
	action->part.ncols = snode->pt_node->ncols;
}

int xrs_allocate_resource(void *hdl, struct alloc_requests *req, void *cb_arg)
{
	struct part_meta *pmp = req->pmp;
	struct xrs_action_load load_act;
	struct solver_node *snode;
	struct solver_state *xrs;
	int ret;

	xrs = (struct solver_state *)hdl;

	ret = sanity_check(pmp, req->rqos, xrs);
	if (ret) {
		dev_err(xrs->cfg.dev, "invalid QoS request");
		return ret;
	}

	if (rg_search_node(&xrs->rgp, req->rid)) {
		dev_err(xrs->cfg.dev, "rid %lld is in-use", req->rid);
		return -EEXIST;
	}

	if (xrs->cfg.total_col - xrs->rgp.allocated < pmp->cdo->ncols)
		return -EBUSY;

	snode = allocate_solver_node(xrs, req);
	if (!snode)
		return -ENOMEM;

	ret = allocate_partition(xrs, pmp, req->rqos, snode);
	if (ret)
		goto free_node;

	fill_load_action(xrs, snode, &load_act);
	ret = xrs->cfg.actions->load(cb_arg, &load_act);
	if (ret)
		goto free_node;

	snode->cb_arg = cb_arg;

	dev_dbg(xrs->cfg.dev, "start col %d\n", snode->pt_node->start_col);

	return 0;

free_node:
	remove_solver_node(&xrs->rgp, snode);

	return ret;
}

int xrs_release_resource(void *hdl, u64 rid)
{
	struct solver_state *xrs = hdl;
	struct solver_node *node;

	node = rg_search_node(&xrs->rgp, rid);
	if (!node) {
		dev_err(xrs->cfg.dev, "node not exist");
		return -ENODEV;
	}

	xrs->cfg.actions->unload(node->cb_arg);
	remove_solver_node(&xrs->rgp, node);

	return 0;
}

void *xrs_init(struct init_config *cfg)
{
	struct solver_rgroup *rgp;
	struct solver_state *xrs;

	xrs = kzalloc(sizeof(*xrs), GFP_KERNEL);
	if (!xrs)
		return NULL;

	memcpy(&xrs->cfg, cfg, sizeof(struct init_config));

	rgp = &xrs->rgp;
	INIT_LIST_HEAD(&rgp->node_list);
	INIT_LIST_HEAD(&rgp->pt_node_list);

	return xrs;
}

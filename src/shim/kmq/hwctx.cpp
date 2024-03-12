// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023-2024, Advanced Micro Devices, Inc. All rights reserved.

#include "hwctx.h"
#include "hwq.h"

#include "core/common/config_reader.h"
#include "core/common/memalign.h"

namespace shim_xdna {

hw_ctx_kmq::
hw_ctx_kmq(const device& device, const xrt::xclbin& xclbin, const xrt::hw_context::qos_type& qos)
  : hw_ctx(device, qos, std::make_unique<hw_q_kmq>(device), xclbin)
{
  auto cu_info = get_cu_info();
  std::vector<amdxdna_cu_config> cu_conf(cu_info.size());

  xcl_bo_flags f = {};
  f.flags = XRT_BO_FLAGS_CACHEABLE;
  for (int i = 0; i < cu_info.size(); i++) {
    auto& ci = cu_info[i];

    m_pdi_bos.push_back(alloc_bo(nullptr, ci.m_pdi.size(), f.all));
    auto& pdi_bo = m_pdi_bos[i];
    auto pdi_vaddr = reinterpret_cast<char *>(
      pdi_bo->map(xrt_core::buffer_handle::map_type::write));
    auto pdi_paddr = pdi_bo->get_properties().paddr;

    auto& cf = cu_conf[i];
    std::memcpy(pdi_vaddr, ci.m_pdi.data(), ci.m_pdi.size());
    cf.xdna_addr = pdi_paddr;
    cf.cu_func = ci.m_func;
  }

#if 0
  for (auto& cf : cu_conf)
    shim_debug("CU_CONF: paddr=%p, func=%d", cf.xdna_addr, cf.cu_func);
#endif

  create_ctx_on_device(cu_conf.data(), cu_conf.size());

  shim_debug("Created KMQ HW context (%d)", get_slotidx());
}

hw_ctx_kmq::
~hw_ctx_kmq()
{
  shim_debug("Destroying KMQ HW context (%d)...", get_slotidx());
}

std::unique_ptr<xrt_core::buffer_handle>
hw_ctx_kmq::
alloc_bo(void* userptr, size_t size, uint64_t flags)
{
  // const_cast: alloc_bo() is not const yet in device class
  auto& dev = const_cast<device&>(get_device());

  // Debug buffer is specific to one context.
  if (xcl_bo_flags{flags}.use == XRT_BO_USE_DEBUG)
    return dev.alloc_bo(userptr, get_slotidx(), size, flags);
  // Other BOs are shared across all contexts.
  return dev.alloc_bo(userptr, INVALID_CTX_HANDLE, size, flags);
}

} // shim_xdna

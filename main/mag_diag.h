#pragma once

#include "bmm350.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 把 BMM350 的现场状态打到日志（在 mag_init 成功后调用一次） */
void mag_diag_dump(struct bmm350_dev *dev);

#ifdef __cplusplus
}
#endif

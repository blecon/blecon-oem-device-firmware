// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#ifndef INFERENCE_H_
#define INFERENCE_H_

#if __cplusplus
extern "C" {
#endif

enum inference_category_t { //lowercase names
    inference_category_idle,
    inference_category_snake,
    inference_category_updown,
    inference_category_wave
};

typedef void (*inference_callback_t)(enum inference_category_t category, float score);

void inference_init(inference_callback_t callback);


#if __cplusplus
}
#endif

#endif

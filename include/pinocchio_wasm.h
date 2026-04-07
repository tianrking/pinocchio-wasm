#ifndef PINOCCHIO_WASM_H
#define PINOCCHIO_WASM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t pino_status_ok(void);
uint8_t *pino_alloc(size_t size);
void pino_dealloc(uint8_t *ptr, size_t size);

void *pino_model_create_from_json(const uint8_t *json_ptr, size_t json_len);
void *pino_model_create_from_urdf(const uint8_t *urdf_ptr, size_t urdf_len);
void *pino_model_create_from_sdf(const uint8_t *sdf_ptr, size_t sdf_len);
void *pino_model_create_from_mjcf(const uint8_t *mjcf_ptr, size_t mjcf_len);
int32_t pino_model_to_json(const void *model, uint8_t **out_json_ptr, size_t *out_json_len);
int32_t pino_model_to_urdf(
    const void *model,
    const uint8_t *robot_name_ptr,
    size_t robot_name_len,
    uint8_t **out_urdf_ptr,
    size_t *out_urdf_len);
int32_t pino_model_to_sdf(
    const void *model,
    const uint8_t *model_name_ptr,
    size_t model_name_len,
    uint8_t **out_sdf_ptr,
    size_t *out_sdf_len);
int32_t pino_model_to_mjcf(
    const void *model,
    const uint8_t *model_name_ptr,
    size_t model_name_len,
    uint8_t **out_mjcf_ptr,
    size_t *out_mjcf_len);
void pino_model_free(void *model);
size_t pino_model_nq(const void *model);
size_t pino_model_nlinks(const void *model);

void *pino_workspace_new(const void *model);
void pino_workspace_free(void *ws);

int32_t pino_aba(
    const void *model,
    void *ws,
    const double *q,
    const double *qd,
    const double *tau,
    const double *gravity_xyz,
    double *qdd_out);

int32_t pino_rnea(
    const void *model,
    void *ws,
    const double *q,
    const double *qd,
    const double *qdd,
    const double *gravity_xyz,
    double *tau_out);

#ifdef __cplusplus
}
#endif

#endif

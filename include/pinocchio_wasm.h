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

int32_t pino_contact_constrained_dynamics(
    const void *model,
    void *ws,
    const double *q,
    const double *qd,
    const double *tau,
    const double *gravity_xyz,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_accel_bias,
    double *qdd_out,
    double *lambda_out);

int32_t pino_contact_constrained_dynamics_batch(
    const void *model,
    void *ws,
    const double *q_batch,
    const double *qd_batch,
    const double *tau_batch,
    size_t batch_size,
    const double *gravity_xyz,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_accel_bias,
    double *qdd_out,
    double *lambda_out);

int32_t pino_apply_contact_impulse(
    const void *model,
    void *ws,
    const double *q,
    const double *qd_minus,
    double restitution,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    double *qd_plus_out,
    double *impulse_out);

int32_t pino_apply_contact_impulse_batch(
    const void *model,
    void *ws,
    const double *q_batch,
    const double *qd_minus_batch,
    size_t batch_size,
    double restitution,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    double *qd_plus_out,
    double *impulse_out);

int32_t pino_contact_jacobian_normal(
    const void *model,
    void *ws,
    const double *q,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    double *jac_out_row_major_kxn);

int32_t pino_contact_constrained_dynamics_friction(
    const void *model,
    void *ws,
    const double *q,
    const double *qd,
    const double *tau,
    const double *gravity_xyz,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_accel_bias,
    const double *contact_friction_coeff,
    double *qdd_out,
    double *lambda_normal_out,
    double *lambda_tangent_out_2k,
    double *force_world_out_3k);

int32_t pino_contact_constrained_dynamics_friction_batch(
    const void *model,
    void *ws,
    const double *q_batch,
    const double *qd_batch,
    const double *tau_batch,
    size_t batch_size,
    const double *gravity_xyz,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_accel_bias,
    const double *contact_friction_coeff,
    double *qdd_out,
    double *lambda_normal_out,
    double *lambda_tangent_out_2k,
    double *force_world_out_3k);

int32_t pino_apply_contact_impulse_friction(
    const void *model,
    void *ws,
    const double *q,
    const double *qd_minus,
    double restitution,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_friction_coeff,
    double *qd_plus_out,
    double *impulse_normal_out,
    double *impulse_tangent_out_2k,
    double *impulse_world_out_3k);

int32_t pino_apply_contact_impulse_friction_batch(
    const void *model,
    void *ws,
    const double *q_batch,
    const double *qd_minus_batch,
    size_t batch_size,
    double restitution,
    size_t num_contacts,
    const int32_t *contact_link_indices_i32,
    const double *contact_points_xyz,
    const double *contact_normals_xyz,
    const double *contact_friction_coeff,
    double *qd_plus_out,
    double *impulse_normal_out,
    double *impulse_tangent_out_2k,
    double *impulse_world_out_3k);

void *pino_collision_model_create(
    size_t num_spheres,
    const int32_t *link_indices,
    const double *centers_xyz,
    const double *radii);

void *pino_collision_model_create_geometries(
    size_t num_geometries,
    const int32_t *geom_types_i32,
    const int32_t *link_indices_i32,
    const double *centers_xyz,
    const double *params_xyz,
    const int32_t *pair_filter_flags_i32x2);

void pino_collision_model_free(void *collision);

int32_t pino_collision_min_distance(
    const void *model,
    const void *collision,
    void *ws,
    const double *q,
    double *distance_out,
    int32_t *pair_out_i32x2);

int32_t pino_collision_min_distance_detailed(
    const void *model,
    const void *collision,
    void *ws,
    const double *q,
    double *distance_out,
    int32_t *pair_out_i32x2,
    double *normal_out_xyz,
    double *point_a_out_xyz,
    double *point_b_out_xyz,
    double *penetration_out,
    int32_t *is_colliding_out);

int32_t pino_collision_min_distance_batch(
    const void *model,
    const void *collision,
    void *ws,
    const double *q_batch,
    size_t batch_size,
    double *distances_out);

int32_t pino_collision_min_distance_detailed_batch(
    const void *model,
    const void *collision,
    void *ws,
    const double *q_batch,
    size_t batch_size,
    double *distances_out,
    double *penetration_out);

int32_t pino_compute_all_terms(
    const void *model,
    void *ws,
    const double *q,
    const double *qd,
    const double *gravity_xyz,
    double *mass_out_row_major,
    double *bias_out,
    double *gravity_out,
    double *coriolis_out,
    double *com_out_xyz,
    double *kinetic_out,
    double *potential_out);

#ifdef __cplusplus
}
#endif

#endif

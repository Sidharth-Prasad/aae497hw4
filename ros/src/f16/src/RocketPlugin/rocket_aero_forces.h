/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int rocket_aero_forces(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void rocket_aero_forces_incref(void);
void rocket_aero_forces_decref(void);
casadi_int rocket_aero_forces_n_out(void);
casadi_int rocket_aero_forces_n_in(void);
const char* rocket_aero_forces_name_in(casadi_int i);
const char* rocket_aero_forces_name_out(casadi_int i);
const casadi_int* rocket_aero_forces_sparsity_in(casadi_int i);
const casadi_int* rocket_aero_forces_sparsity_out(casadi_int i);
int rocket_aero_forces_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* rocket_aero_forces_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
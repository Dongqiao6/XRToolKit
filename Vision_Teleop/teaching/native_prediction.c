/* Computation-only copy of BareSafety's fixed interval. No control policy.
   Pinned MuJoCo headers/DLL, no fast-math. Pair order is lexicographic. */
#include <mujoco/mujoco.h>
#include <math.h>
#include <stdlib.h>

typedef struct { double distance, margin; int a,b; } Inspection;
typedef struct {
  Inspection command, aggregate;
  double command_speed, control_margin, max_speed;
  int samples, reason;
} Result;

__declspec(dllexport) int prediction_version(void) { return mj_version(); }

static Inspection inspect(const mjModel *m, const mjData *d,
                          const int *pairs,int np,const int *qi) {
  Inspection out={.03,INFINITY,-1,-1};
  for(int k=0;k<np;k++) {
    int a=pairs[2*k],b=pairs[2*k+1];
    double x=d->geom_xpos[3*a]-d->geom_xpos[3*b];
    double y=d->geom_xpos[3*a+1]-d->geom_xpos[3*b+1];
    double z=d->geom_xpos[3*a+2]-d->geom_xpos[3*b+2];
    double ra=m->geom_rbound[a],rb=m->geom_rbound[b];
    if(ra>0 && rb>0 && sqrt((x*x+y*y)+z*z)>ra+rb+.03)continue;
    double distance=mj_geomDistance(m,d,a,b,.03,NULL);
    if(out.a<0 || distance<out.distance) {
      out.distance=distance;out.a=a;out.b=b;
    }
  }
  for(int j=0;j<m->nu;j++) {
    int id=m->actuator_trnid[2*j];double q=d->qpos[qi[j]];
    double v=fmin(q-m->jnt_range[2*id],m->jnt_range[2*id+1]-q);
    out.margin=fmin(out.margin,v);
  }
  return out;
}

/* Reason: 1 command, 2 clearance, 3 joint range, 4 actual speed, 5 nonfinite. */
static int check(const mjModel *m,const mjData *d,const int *pairs,int np,
                 const int *qi,Result *out) {
  Inspection v=inspect(m,d,pairs,np,qi);double speed=0;
  for(int j=0;j<m->nq;j++)if(!isfinite(d->qpos[j]))return 5;
  for(int j=0;j<m->nv;j++) {
    if(!isfinite(d->qvel[j]))return 5;
    speed=fmax(speed,fabs(d->qvel[j]));
  }
  out->samples++;
  if(v.distance<out->aggregate.distance) {
    out->aggregate.distance=v.distance;out->aggregate.a=v.a;out->aggregate.b=v.b;
  }
  out->aggregate.margin=fmin(out->aggregate.margin,v.margin);
  out->max_speed=fmax(out->max_speed,speed);
  return v.distance<.005-1e-9 ? 2 : v.margin < -1e-9 ? 3 : speed>.6+1e-7 ? 4 : 0;
}

__declspec(dllexport) void prediction_interval(
    const mjModel *m,const mjData *source,mjData *scratch,mjData *trial,
    const int *pairs,int np,const int *qi,const double *command,double dt,
    int steps,double *old,Result *out) {
  *out=(Result){0};out->aggregate=(Inspection){.03,INFINITY,-1,-1};
  for(int j=0;j<m->nq;j++)scratch->qpos[j]=command[j];
  mj_kinematics(m,scratch);out->command=inspect(m,scratch,pairs,np,qi);
  out->control_margin=INFINITY;
  for(int j=0;j<m->nu;j++) {
    double ctrl=command[qi[j]];
    out->control_margin=fmin(out->control_margin,
      fmin(ctrl-m->actuator_ctrlrange[2*j],m->actuator_ctrlrange[2*j+1]-ctrl));
    out->command_speed=fmax(out->command_speed,fabs(ctrl-source->ctrl[j])/dt);
  }
  if(out->command.distance<.005-1e-9 || out->command.margin< -1e-9 ||
     out->control_margin< -1e-9 || out->command_speed>.6+1e-7) {out->reason=1;return;}
  mj_copyData(trial,m,source);
  for(int j=0;j<m->nu;j++)trial->ctrl[j]=command[qi[j]];
  for(int k=0;k<steps;k++) {
    for(int j=0;j<m->nq;j++)old[j]=trial->qpos[j];
    mj_step(m,trial);mj_kinematics(m,trial);
    for(int j=0;j<m->nq;j++)scratch->qpos[j]=(old[j]+trial->qpos[j])/2;
    for(int j=0;j<m->nv;j++)scratch->qvel[j]=trial->qvel[j];
    mj_kinematics(m,scratch);
    out->reason=check(m,scratch,pairs,np,qi,out);if(out->reason)return;
    out->reason=check(m,trial,pairs,np,qi,out);if(out->reason)return;
  }
}

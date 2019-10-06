/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "RocketPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)

////////////////////////////////////////////////////////////////////////////////
RocketPlugin::RocketPlugin():
	_double_this(double_this_functions()),
  _rocket_aero_forces(rocket_aero_forces_functions()),
  _rocket_aero_moment(rocket_aero_moment_functions()),
  _rocket_prop_forces(rocket_prop_forces_functions()),
  _rocket_prop_moment(rocket_prop_moment_functions())
{
  
}

/////////////////////////////////////////////////
RocketPlugin::~RocketPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool RocketPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool RocketPlugin::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::LinkPtr &_link)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string linkName = _sdf->Get<std::string>(_sdfParam);
  _link = this->model->GetLink(linkName);
  if (!_link)
  {
    gzerr << "Failed to find link [" << linkName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void RocketPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "RocketPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "RocketPlugin _sdf pointer is NULL");
  this->model = _model;

  // Find motor link to apply propulsion forces
  if (!this->FindLink("motor", _sdf, this->motor)) {
    GZ_ASSERT(false, "RocketPlugin failed to find motor");
  }

  // Find body link to apply propulsion forces
  if (!this->FindLink("body", _sdf, this->body)) {
    GZ_ASSERT(false, "RocketPlugin failed to find body");
  }

  // Update time.
  this->lastUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&RocketPlugin::Update, this, std::placeholders::_1));

  gzlog << "Rocket ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void RocketPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastUpdateTime)
  {
    double dt = (curTime - this->lastUpdateTime).Double();

    auto motor_inertial = this->motor->GetInertial();
    float m_fuel = motor_inertial->Mass();
    auto body_inertial = this->body->GetInertial();
    float m = body_inertial->Mass();
    float m_dot = .1;
    float m_empty = 0.2;
    m = m - m_dot*dt;
	  float P0 = 101325.0; // freestream pressure
	  float Pe = 1.0*P0; // disable effect for now
    if (m < m_empty) {
      m = m_empty;
      m_dot = 0;
	  Pe = P0;
    }
	
    float r = 0.1; // nozzle radius
	  float A = M_PI*r*r; // nozzle exit area
    float ve = 320; // exit velocity, m/s, guess

    float g = 9.81;
    float Jx = 1.0;
    float Jy = 1.0;
    float Jz = 1.0;
    float Jxz = 0.1;
    float l_fin = 1.0;
    float cl_alpha = 2*3.14159265;
    float cl0 = 0;
    float cd0 = 0.01;
    float K = 0.01;
    float S_fin = 0.05;
    float rho = 1.225;
    float l_motor = 1.0;

    double p[15] = { g, Jx, Jy, Jz, Jxz, ve, l_fin, cl_alpha, cl0, cd0, K, S_fin, rho, m_empty, l_motor};


    double FA_b[3] = {0,0,0};
    double MA_b[3] = {0,0,0};
    double FP_b[3] = {0,0,0};
    double MP_b[3] = {0,0,0};
    
  ignition::math::Pose3d Pn0 = this->body->WorldInertialPose();
    
    // Get current states from gazebo
    auto pose_world_inertia = this->body->WorldInertialPose();  //pose in world inertia frame ????
    auto pos_ine            = pose_world_inertia.Pos();         // [x,y,z], use [] to access elements
    auto rot_ine            = pose_world_inertia.Rot();         // [w x y z], quaternions. use .qw .qx .qy .qz to access.

    auto omega_ine          = this->body->WorldAngularVel();    // inertial angular velocity
    auto vel_ine            = this->body->WorldLinearVel();     // inertial linaer velocity
    
    // std::cout<<"\n Inertial Velocity"<<std::endl;
    // std::cout<<vel_ine<<std::endl;

    // Gazebo uses xyz coord.
    // rocket uses NED coord.
    // z->-D, 


    double pos_n[3]   = {pos_ine.X()  ,pos_ine.Y()  ,pos_ine.Z()  };
    double q_ENU_TRF[4]       = {rot_ine.W()  ,rot_ine.X()  ,rot_ine.Y()  ,rot_ine.Z()};
    double vel_ENU[3]   = {vel_ine.X()  ,vel_ine.Y()  ,vel_ine.Z()  };
    double omg_ENU[3] = {omega_ine.X(),omega_ine.Y(),omega_ine.Z()};

    //Convert velocities wrt inertial frame to velocities wrt body frame
    //(It is also necessary to convert orientation from world to local csys using modified rodrigues parameters)
    
    //Fill x array with states (wrt body frame)
    //{omg_b, r_nb, v_b, pos_n, m_fuel}
    double x_ENU[14]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::copy(omg_ENU, omg_ENU+3, x_ENU   );
    std::copy(q_ENU_TRF , q_ENU_TRF+4, x_ENU+3 );
    std::copy(vel_ENU  , vel_ENU  +3, x_ENU+7 );
    std::copy(pos_n, pos_n+3, x_ENU+10);
    x_ENU[13] = m_fuel;



    _rocket_aero_forces.arg(2,p); //Set rocket constant parameters for each eom
    _rocket_aero_moment.arg(2,p); //Set rocket constant parameters for each eom
    _rocket_prop_forces.arg(2,p); //Set rocket constant parameters for each eom
    _rocket_prop_moment.arg(2,p); //Set rocket constant parameters for each eom

    _rocket_aero_forces.arg(0,x_ENU); //Set address of rocket state for each eom
    _rocket_aero_moment.arg(0,x_ENU); //Set address of rocket state for each eom
    _rocket_prop_forces.arg(0,x_ENU); //Set address of rocket state for each eom
    _rocket_prop_moment.arg(0,x_ENU); //Set address of rocket state for each eom


    double u[4]   = {0.1,0,0,0};     //Set input to zero for now
    _rocket_aero_forces.arg(1,u); 
    _rocket_aero_moment.arg(1,u); 
    _rocket_prop_forces.arg(1,u); 
    _rocket_prop_moment.arg(1,u);

    _rocket_aero_forces.res(0,FA_b); //Set rocket aero forces
    _rocket_aero_moment.res(0,MA_b); //Set rocket aero moments
    _rocket_prop_forces.res(0,FP_b); //Set rocket propulsion forces
    _rocket_prop_moment.res(0,MP_b); //Set rocket propulsion moments

    _rocket_aero_forces.eval(); //Calculate new value of FA_b
    _rocket_aero_moment.eval(); //Calculate new value of MA_b
    _rocket_prop_forces.eval(); //Calculate new value of FP_b
    _rocket_prop_moment.eval(); //Calculate new value of MP_b
 

    // FA_b[0] = 0;
    // FA_b[1] = 0;
    // FA_b[2] = 0;
    // MA_b[0] = 0;
    // MA_b[1] = 0;
    // MA_b[2] = 0;

    // FP_b[3] = {0,0,0};
    // MP_b[3] = {0,0,0};


    // this->motor->AddRelativeForce (ignition::math::Vector3d(0,0,300));
    this->motor->AddRelativeForce (ignition::math::Vector3d(FP_b[2], FP_b[1], FP_b[0]));
    gzdbg << "FP_b[0]=" << FP_b[0] << " FP_b[1]=" << FP_b[1] << " FP_b[2]=" << FP_b[2] << "\n" << std::endl;
    this->body ->AddRelativeForce (ignition::math::Vector3d(FA_b[0], FA_b[1], FA_b[2]));
    gzdbg << "FA_b[0]=" << FA_b[0] << " FA_b[1]=" << FA_b[1] << " FA_b[2]=" << FA_b[2] << "\n" << std::endl;
    this->motor->AddRelativeTorque(ignition::math::Vector3d(MP_b[0], MP_b[1], MP_b[2]));
    this->body ->AddRelativeTorque(ignition::math::Vector3d(MA_b[0], MA_b[1], MA_b[2]));

	  // std::cout << "input" << u << "result" <<  << std::endl;
    std::cout << "velocity" << vel_ENU << std::endl;
  }
}

/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rtps.c
 * RTPS to uORB bridge
 *
 * @author Kevin Mehall <kevin.mehall@3drobotics.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/optical_flow.h>
#include <lib/conversion/rotation.h>
#include <uORB/topics/input_rc.h>
#include <drivers/drv_rc_input.h>
#include <redrider.capnp.h>

#include <commkit/node.h>
#include <commkit/subscriber.h>

static volatile bool keep_running;
static volatile bool started = false;

struct ORB_ToRTPS_Base {
  commkit::PublisherPtr publisher;
  const orb_metadata* orb_id;
  int orb_fd;
  
  virtual void poll_matched() {}
  
  virtual ~ORB_ToRTPS_Base() {}
};

template<typename CapnType, typename orb_type>
struct ORB_ToRTPS: ORB_ToRTPS_Base {
  ORB_ToRTPS(commkit::Node& node, const char* rtps_topic, bool reliable, const orb_metadata* orb) {
    auto topic = commkit::Topic::capn<CapnType>(rtps_topic);
    commkit::PublicationOpts pub_opts;
    pub_opts.reliable = reliable;
    pub_opts.history = 0;
    
    publisher = node.createPublisher(topic);
    if (publisher == nullptr) {
        warnx("Error creating publisher for %s\n", rtps_topic);
        return;
    }
    if (!publisher->init(pub_opts)) {
        warnx("Error initializing publisher for %s\n", rtps_topic);
        return;
    }
    
    orb_id = orb;
    orb_fd = orb_subscribe(orb);
  }
  
  void poll_matched() {
    orb_type data;
    orb_copy(orb_id, orb_fd, &data);
    on_orb(data);
  }
  
  virtual void on_orb(orb_type&) {}
};

struct ORB_FromRTPS_Base {
  commkit::SubscriberPtr subscriber;
  const orb_metadata* orb_id;
  orb_advert_t _orb_pub;
  
  virtual ~ORB_FromRTPS_Base() {}
};

template<typename CapnType, typename orb_type>
struct ORB_FromRTPS: ORB_FromRTPS_Base {
  ORB_FromRTPS(commkit::Node& node, const char* rtps_topic, bool reliable, const orb_metadata* orb) {
    auto topic = commkit::Topic::capn<CapnType>(rtps_topic);
    commkit::SubscriptionOpts sub_opts;
    sub_opts.reliable = reliable;
    sub_opts.history = 0;
    
    subscriber = node.createSubscriber(topic);
    if (subscriber == nullptr) {
        warnx("Error creating subscriber for %s\n", rtps_topic);
        return;
    }
    subscriber->onMessage.connect(&ORB_FromRTPS::rtps_recv, this);
    if (!subscriber->init(sub_opts)) {
        warnx("Error initializing subscriber for %s\n", rtps_topic);
        return;
    }
    
    orb_id = orb;
    _orb_pub = nullptr;
  }
  
  virtual void on_rtps(typename CapnType::Reader) {}
  
  void rtps_recv(commkit::SubscriberPtr sub) {
    commkit::Payload payload;
    while (sub->take(&payload)) {
      auto capn = payload.toReader<CapnType>();
      on_rtps(capn);
    }
  }
  
  void publish_orb(orb_type& data) {
    if (_orb_pub == nullptr) {
      _orb_pub = orb_advertise(orb_id, &data);
    } else {
      orb_publish(orb_id, _orb_pub, &data);
    }
  } 
};

struct RcChannels_FromRTPS: ORB_FromRTPS<redrider::RcChannels, input_rc_s> {
  RcChannels_FromRTPS(commkit::Node& node): ORB_FromRTPS(node, "RcChannels", false, ORB_ID(input_rc)) {}
    
  void on_rtps(redrider::RcChannels::Reader reader) {
    struct input_rc_s rc = {};
    rc.timestamp_publication = hrt_absolute_time();
    rc.timestamp_last_signal = rc.timestamp_publication;
    rc.channel_count = 8;
    rc.rc_failsafe = false;
    rc.rc_lost = false;
    rc.rc_lost_frame_count = 0;
    rc.rc_total_frame_count = 1;
    rc.rc_ppm_frame_length = 0;
    rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
    rc.rssi = RC_INPUT_RSSI_MAX;
    
    /* channels */
    rc.values[0] = reader.getCh1();
    rc.values[1] = reader.getCh2();
    rc.values[2] = reader.getCh3();
    rc.values[3] = reader.getCh4();
    rc.values[4] = reader.getCh5();
    rc.values[5] = reader.getCh6();
    rc.values[6] = reader.getCh7();
    rc.values[7] = reader.getCh8();
    
    publish_orb(rc);
  }
};

struct OpticalFlow_FromRTPS: ORB_FromRTPS<redrider::OpticalFlow, optical_flow_s> {
  OpticalFlow_FromRTPS(commkit::Node& node): ORB_FromRTPS(node, "OpticalFlow", false, ORB_ID(optical_flow)) {}
  
  void on_rtps(redrider::OpticalFlow::Reader reader) {
      enum Rotation flow_rot;
      param_get(param_find("SENS_FLOW_ROT"), &flow_rot);

      struct optical_flow_s f;
      memset(&f, 0, sizeof(f));

      f.timestamp = reader.getTimestamp() / 1000;
      f.integration_timespan = reader.getIntegrationTime();
      auto flowIntegral = reader.getFlowIntegral();
      f.pixel_flow_x_integral = flowIntegral.getX();
      f.pixel_flow_y_integral = flowIntegral.getY();
      auto gyroIntegral = reader.getGyroIntegral();
      f.gyro_x_rate_integral = gyroIntegral.getX();
      f.gyro_y_rate_integral = gyroIntegral.getY();
      f.gyro_z_rate_integral = gyroIntegral.getZ();
      f.quality = reader.getQuality();
      f.time_since_last_sonar_update = 0;
      f.ground_distance_m = NAN;;
      f.sensor_id = 0;
      f.gyro_temperature = 0;

      /* rotate measurements according to parameter */
      float zeroval = 0.0f;
      rotate_3f(flow_rot, f.pixel_flow_x_integral, f.pixel_flow_y_integral, zeroval);
      rotate_3f(flow_rot, f.gyro_x_rate_integral, f.gyro_y_rate_integral, f.gyro_z_rate_integral);

      publish_orb(f);
  }
};

struct Gyro_ToRTPS: ORB_ToRTPS<redrider::Gyro, sensor_gyro_s> {
  Gyro_ToRTPS(commkit::Node& node): ORB_ToRTPS(node, "Gyro", false, ORB_ID(sensor_gyro)) {}
  
  void on_orb(sensor_gyro_s& gyro) {
    capnp::MallocMessageBuilder mb;
    auto p_gyro = mb.getRoot<redrider::Gyro>();
    p_gyro.setTimestamp(gyro.timestamp * 1000);
    p_gyro.getIntegral().setX(gyro.x_integral);
    p_gyro.getIntegral().setY(gyro.y_integral);
    p_gyro.getIntegral().setZ(gyro.x_integral);
    p_gyro.setIntegrationTime(gyro.integral_dt);
    publisher->publish(mb);
  }
};

struct CurrentAttitude_ToRTPS: ORB_ToRTPS<redrider::CurrentAttitude, vehicle_attitude_s> {
  CurrentAttitude_ToRTPS(commkit::Node& node): ORB_ToRTPS(node, "CurrentAttitude", false, ORB_ID(vehicle_attitude)) {}
  
  void on_orb(vehicle_attitude_s& att) {
    capnp::MallocMessageBuilder mb;
    auto p = mb.getRoot<redrider::CurrentAttitude>();
    p.setTimestamp(att.timestamp * 1000);
    p.setRoll(att.roll);
    p.setPitch(att.pitch);
    p.setYaw(att.yaw);
    p.setRollSpeed(att.rollspeed);
    p.setPitchSpeed(att.pitchspeed);
    p.setYawSpeed(att.yawspeed);
    publisher->publish(mb);
  }
};

struct CurrentLocalPosition_ToRTPS: ORB_ToRTPS<redrider::CurrentLocalPosition, vehicle_local_position_s> {
  CurrentLocalPosition_ToRTPS(commkit::Node& node): ORB_ToRTPS(node, "CurrentLocalPosition", false, ORB_ID(vehicle_local_position)) {}
  
  void on_orb(vehicle_local_position_s& pos) {
    capnp::MallocMessageBuilder mb;
    auto p = mb.getRoot<redrider::CurrentLocalPosition>();
    p.setTimestamp(pos.timestamp * 1000);
    auto position = p.getPosition();
    position.setX(pos.x);
    position.setY(pos.y);
    position.setZ(pos.z);
    auto velocity = p.getVelocity();
    velocity.setX(pos.vx);
    velocity.setY(pos.vy);
    velocity.setZ(pos.vz);
    publisher->publish(mb);
  }
};

struct VehicleStatus_ToRTPS: ORB_ToRTPS<redrider::VehicleStatus, vehicle_status_s> {
  VehicleStatus_ToRTPS(commkit::Node& node): ORB_ToRTPS(node, "VehicleStatus", false, ORB_ID(vehicle_status)) {}

  void on_orb(vehicle_status_s& status) {
    capnp::MallocMessageBuilder mb;
    auto p = mb.getRoot<redrider::VehicleStatus>();
    
    p.setArmed(status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
           || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);
           
    switch (status.nav_state) {
      case vehicle_status_s::NAVIGATION_STATE_MANUAL:
        p.setMode(redrider::VehicleStatus::PX4Mode::MANUAL);
        break;

      case vehicle_status_s::NAVIGATION_STATE_ACRO:
        p.setMode(redrider::VehicleStatus::PX4Mode::ACRO);
        break;

      case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
        p.setMode(redrider::VehicleStatus::PX4Mode::RATTITUDE);
        break;

      case vehicle_status_s::NAVIGATION_STATE_STAB:
        p.setMode(redrider::VehicleStatus::PX4Mode::STAB);
        break;

      case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
        p.setMode(redrider::VehicleStatus::PX4Mode::ALTCTL);
        break;

      case vehicle_status_s::NAVIGATION_STATE_POSCTL:
        p.setMode(redrider::VehicleStatus::PX4Mode::POSCTL);
        break;

      case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_TAKEOFF);
        break;

      case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_MISSION);
        break;

      case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_LOITER);
        break;
        
      case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_FOLLOW_TARGET);
        break;
        
      case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: /* fallthrough */
      case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_RTL);
        break;

      case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
      case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
      case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL: /* fallthrough */
      case vehicle_status_s::NAVIGATION_STATE_DESCEND:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_LAND);
        break;

      case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
        p.setMode(redrider::VehicleStatus::PX4Mode::AUTO_RTGS);
        break;

      case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
        p.setMode(redrider::VehicleStatus::PX4Mode::TERMINATION);
        break;

      case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
        p.setMode(redrider::VehicleStatus::PX4Mode::OFFBOARD);
        break;

      case vehicle_status_s::NAVIGATION_STATE_MAX:
        /* this is an unused case, ignore */
        break;
    }
        
    publisher->publish(mb);
  }
};



static int thread_main() {
  commkit::Node node;
  if (!node.init("px4")) {
      warnx("Error initializing commkit\n");
      return 1;
  }
  
  std::unique_ptr<ORB_ToRTPS_Base> to_rtps[] = {
    std::unique_ptr<ORB_ToRTPS_Base>(new Gyro_ToRTPS(node)),
    std::unique_ptr<ORB_ToRTPS_Base>(new CurrentAttitude_ToRTPS(node)),
    std::unique_ptr<ORB_ToRTPS_Base>(new CurrentLocalPosition_ToRTPS(node)),
    std::unique_ptr<ORB_ToRTPS_Base>(new VehicleStatus_ToRTPS(node)),
  };
  
  std::unique_ptr<ORB_FromRTPS_Base> from_rtps[] = {
    std::unique_ptr<ORB_FromRTPS_Base>(new OpticalFlow_FromRTPS(node)),
    std::unique_ptr<ORB_FromRTPS_Base>(new RcChannels_FromRTPS(node)),
  };
  
  const size_t to_rtps_count = sizeof(to_rtps) / sizeof(to_rtps[0]);
  px4_pollfd_struct_t fds[to_rtps_count];
  
  for (size_t i = 0; i<to_rtps_count; i++) {
    fds[i] = { .fd = to_rtps[i]->orb_fd, .events = POLLIN };
  }
  
  while (keep_running) {
    px4_poll(fds, to_rtps_count, 1000);
    for (size_t i = 0; i<to_rtps_count; i++) {
      if (fds[i].revents & POLLIN) {
        to_rtps[i]->poll_matched();
      }
    }
  }
  
  (void) from_rtps;

  started = false;
  return OK;
}

static int start() {
  if (started) {
    warnx("Already running");
    return 1;
  }

  started = true;
  keep_running = true;
  px4_task_spawn_cmd("rtps",
         SCHED_DEFAULT,
         SCHED_PRIORITY_DEFAULT,
         2700,
         (px4_main_t)&thread_main,
         0);

  return OK;
}

static int stop() {
  keep_running = false;
  while (started) usleep(10000);
  return OK;
}

extern "C" __EXPORT int rtps_main(int argc, char *argv[]);

extern "C" int rtps_main(int argc, char *argv[])
{
    if (!strcmp(argv[1], "start")) {
      return start();

    } else if (!strcmp(argv[1], "stop")) {
      return stop();

    } else {
      	warnx("usage: rtps {start|stop}");
        return 1;
    }

    return OK;
}

#include "motion_primitive.hpp"

#define PRECISION_ACC_CHANGE (1.0e-3)

Primitive MotionPrimitive::fix_rotation_continuity(Primitive &old,
                                                   const Drone *drone) {
  INFO("fix_rotation_continuity begin");
  MotionPrimitive::check_primitive(old);
  INFO_VAR(old.rotations.size());
  INFO_VAR(old.translations.size());
  Primitive fixed;
  fixed.rotations.push_back(old.rotations[0]);

  for (int var = 1; var < old.rotations.size(); ++var) {
    TrRMaxAcc3D rot_before = fixed.rotations[var - 1];
    const DroneState first_rot_end = rot_before.get_end_state();
    const Quaternion first_rot_end_q = first_rot_end.getAttitude();

    TrMaxAcc3D translation = old.translations[var - 1];
    const DroneState first_trans_beg = translation.get_start_state();
    const Quaternion first_trans_beg_q = first_trans_beg.getAttitude();

    if ((first_rot_end_q.coeffs() - first_trans_beg_q.coeffs()).norm() >
        PRECISION) {
      translation.add_quaternion_for_time(0, first_rot_end_q);
    }

    fixed.translations.push_back(translation);


    const DroneState first_trans_end = translation.get_end_state();
    const Quaternion first_trans_end_q = first_trans_end.getAttitude();

    TrRMaxAcc3D rot_after = old.rotations[var];

    const DroneState second_rot_beg = rot_after.get_start_state();
    const Quaternion second_rot_beg_q = second_rot_beg.getAttitude();
    const DroneState second_rot_end = rot_after.get_end_state();
    const Quaternion second_rot_end_q = second_rot_end.getAttitude();
    // const Quaternion second_rot_end_q = second_rot_beg.getAttitude();

    if ((first_trans_end_q.coeffs() - second_rot_beg_q.coeffs()).norm() >
        PRECISION) {
      rot_after =
        calc_rot_trajectories(first_trans_end_q, second_rot_end_q,
                              first_trans_end, Vector<3>::Zero(), drone);
      rot_after.gate_idx = old.rotations[var].gate_idx;
    }
    fixed.rotations.push_back(rot_after);
  }

  //  INFO_VAR(fixed.rotations.size());
  //  INFO_VAR(fixed.translations.size());
  fixed.time = fixed.calc_time();
  // exit(1);
  INFO("fix_rotation_continuity end");
  return fixed;
}

void MotionPrimitive::check_primitive(Primitive &pr, bool check_quaternions) {
  // debug checking of z axis continuity begin
  for (int var = 1; var < pr.rotations.size(); ++var) {
    // INFO("rot id " << var - 1);
    // INFO(pr.rotations[var - 1]);
    // INFO("trans " << var - 1);
    // INFO(pr.translations[var - 1]);
    // INFO("rot " << var);
    // INFO(pr.rotations[var]);
    if (!pr.rotations[var - 1].exists()) {
      INFO("rotation " << var - 1 << " does not exists")
      exit(1);
    }
    if (!pr.translations[var - 1].exists()) {
      INFO("translations " << var << " does not exists")
      exit(1);
    }

    const DroneState first_rot_end = pr.rotations[var - 1].get_end_state();
    const Quaternion first_rot_end_q = first_rot_end.getAttitude();

    const DroneState first_trans_beg =
      pr.translations[var - 1].get_start_state();
    const Quaternion first_trans_beg_q = first_trans_beg.getAttitude();


    const Vector<3> z1 = first_rot_end_q * Vector<3>(0, 0, 1);
    const Vector<3> z2 = first_trans_beg_q * Vector<3>(0, 0, 1);
    if ((z1 - z2).norm() > PRECISION) {
      INFO_VAR(z1.transpose());
      INFO_VAR(z2.transpose());
      INFO("bad thrust dir quaternion between rot id "
           << var - 1 << " and trans id " << var - 1);
      exit(1);
    }

    if (pr.rotations[var - 1].time() == 0) {
      INFO_RED("rotation time of " << var - 1 << " is zero!!!!!")
    }

    if (check_quaternions) {
      if ((first_rot_end_q.coeffs() - first_trans_beg_q.coeffs()).norm() >
          PRECISION) {
        INFO_VAR(first_rot_end_q.coeffs().transpose());
        INFO_VAR(first_trans_beg_q.coeffs().transpose());
        INFO("bad quaternion between rot " << var - 1 << " and trans "
                                           << var - 1);
        exit(1);
      }
    }

    if (!pr.rotations[var].exists()) {
      INFO("rotation " << var << " does not exists")
      exit(1);
    }

    const DroneState first_trans_end = pr.translations[var - 1].get_end_state();
    const Quaternion first_trans_end_q = first_trans_end.getAttitude();

    const DroneState second_rot_beg = pr.rotations[var].get_start_state();
    const Quaternion second_rot_beg_q = second_rot_beg.getAttitude();

    const Vector<3> z3 = first_trans_end_q * Vector<3>(0, 0, 1);
    const Vector<3> z4 = second_rot_beg_q * Vector<3>(0, 0, 1);
    if ((z3 - z4).norm() > PRECISION) {
      INFO_VAR(z3.transpose());
      INFO_VAR(z4.transpose());
      INFO("bad thrust dir quaternion between trans " << var - 1 << " and rot "
                                                      << var);
      exit(1);
    }

    if (pr.rotations[var].time() == 0) {
      INFO_RED("rotation time of " << var << " is zero!!!!!")
    }

    if (check_quaternions) {
      if ((first_trans_end_q.coeffs() - second_rot_beg_q.coeffs()).norm() >
          PRECISION) {
        INFO_VAR(first_trans_end_q.coeffs().transpose());
        INFO_VAR(second_rot_beg_q.coeffs().transpose());
        INFO("bad quaternion between trans " << var - 1 << " and rot " << var);
        exit(1);
      }
    }
  }

  /*
  INFO("rot " << 0);
  INFO(pr.rotations[0]);
  INFO("rot_end_state " << pr.rotations[0].get_end_state());
  for (int var = 0; var < pr.translations.size(); ++var) {
    INFO("trans " << var);
    DroneState end_state_tr = pr.translations[var].get_end_state();
    INFO(pr.translations[var]);
    INFO("end_state_tr " << end_state_tr);
    INFO("rot " << var + 1);
    INFO(pr.rotations[var + 1]);
    INFO("rot_end_state " << pr.rotations[var + 1].get_end_state());
  }
  */

  // debug checking of z axis continuity end
}

Primitive MotionPrimitive::acc_primitive(const DroneState &from_state,
                                         const DroneState &to_state,
                                         const Drone *drone) {
  // INFO("----------------------------------------------------------------------")
  std::vector<TrMaxAcc3D> profile_trans;
  std::vector<TrRMaxAcc3D> profile_rot;

  const Vector<3> omega_zero(0, 0, 0);

  TrMaxAcc3D xyz = calc_max_acc_thrust(from_state, to_state, drone, 0.001);

  // const double max_time = xyz.time();
  // for (size_t axi = 0; axi < 3; axi++) {
  //   Tr1D scaled =
  //     one_dim_double_integrator_lim_vel(xyz.get_axis(axi), max_time);
  //   xyz.set_by_axis(axi, scaled);
  // }

  INFO("xyz acc model " << xyz);


  const Vector<3> acc_init = xyz.get_start_state().a;
  // const Vector<3> acc_init(xyz.x.a1, xyz.y.a1, xyz.z.a1);
  const Vector<3> acc_end = xyz.get_end_state().a;
  // const Vector<3> acc_end(xyz.x.a2, xyz.y.a2, xyz.z.a2);
  const Vector<3> thrust_dir_init = acc_init - GVEC;
  const Vector<3> thrust_dir_end = acc_end - GVEC;


  INFO("thrust_dir_init " << thrust_dir_init.normalized())
  INFO("thrust_dir_end " << thrust_dir_end.normalized())
  Quaternion q_init_target_thrust =
    Quaternion::FromTwoVectors(Vector<3>(0, 0, 1), thrust_dir_init);
  // INFO("q_target_thrust " << q_init_target_thrust.w() << " "
  //                       << q_init_target_thrust.vec().transpose());


  // debug part begin
  const Vector<3> test = q_init_target_thrust * Vector<3>(0, 0, 1);

  if ((test - thrust_dir_init.normalized()).norm() > PRECISION) {
    INFO_VAR(thrust_dir_init.transpose());
    INFO_VAR(test.transpose());
    INFO("bad thrust dir quaternion")
    exit(1);
  }
  // debug part end

  // initial rotation toward acc
  const Quaternion q_from = from_state.getAttitude();
  // INFO("tr_roll_pitch_start");
  TrRMaxAcc3D tr_roll_pitch_start = calc_rot_trajectories(
    q_from, q_init_target_thrust, from_state, omega_zero, drone);

  profile_rot.push_back(tr_roll_pitch_start);
  // INFO("tr_roll_pitch_start");
  // INFO(tr_roll_pitch_start)


  // struct for switching acceleration
  struct aswitch {
    Scalar time;
    Scalar time_end;  // if the switch is merged then the time_end time is
                      // different from the time
    int axis;
    Quaternion q;
    TrRMaxAcc3D rot;
    static bool compare(aswitch i, aswitch j) { return (i.time < j.time); };
  };

  // fill the switches times
  std::vector<aswitch> a_switches;
  aswitch swbeg;
  swbeg.time_end = swbeg.time = 0;
  swbeg.axis = -1;
  swbeg.rot = tr_roll_pitch_start;
  swbeg.q = q_init_target_thrust;
  a_switches.push_back(swbeg);
  for (int i = 0; i < 3; ++i) {
    const Tr1D axis = xyz.get_axis(i);
    if (axis.t1 > 0 and axis.t3 > 0 and fabs(axis.a1) > PRECISION_ACC_CHANGE) {
      aswitch sw;
      sw.time_end = sw.time = axis.t1;
      sw.axis = i;
      a_switches.push_back(sw);
    }
  }
  std::sort(a_switches.begin(), a_switches.end(),
            aswitch::compare);  // sort the switches based on time
  // INFO_VAR(a_switches.size());
  // INFO("a_switches[0].q " << a_switches[0].q.w() << " " <<
  // a_switches[0].q.x()
  //                         << " " << a_switches[0].q.y() << " "
  //                         << a_switches[0].q.z());

  // fill the switches accelerations and get the respective attitudes
  Vector<3> acc_before = thrust_dir_init;
  for (int i = 1; i < a_switches.size(); ++i) {
    int ax = a_switches[i].axis;
    acc_before(ax) = thrust_dir_end(ax);
    // const Vector<3> thrust_dir_switch =
    //  Drone::thrust_acc_from_body_acc(acc_before);
    // INFO_VAR(acc_before.transpose())
    const Quaternion q_thrust_dir =
      Quaternion::FromTwoVectors(Vector<3>(0, 0, 1), acc_before);
    a_switches[i].q = q_thrust_dir;
  }

  // just for debugging print the individual swithces
  for (int var = 0; var < a_switches.size(); ++var) {
    INFO(var << " switch " << a_switches[var].axis << " time "
             << a_switches[var].time << " q " << a_switches[var].q.w() << " "
             << a_switches[var].q.vec().transpose());
  }


  // calculate rotational switches when the acceleration changes
  for (int i_sw = 1; i_sw < a_switches.size(); ++i_sw) {
    INFO("------------------------------------------------")
    // INFO_VAR(i_sw)
    const Quaternion q_from = a_switches[i_sw - 1].q;
    const Quaternion q_to = a_switches[i_sw].q;
    const Scalar t_switch = a_switches[i_sw].time;
    const DroneState state_in_switch = xyz.state_in_time(t_switch);
    INFO("\tq_from " << q_from.w() << " " << q_from.vec().transpose());
    INFO("\tq_to " << q_to.w() << " " << q_to.vec().transpose());
    // INFO("switch time " << t_switch);
    // INFO("rot tr " << i_sw);
    TrRMaxAcc3D rot_tr =
      calc_rot_trajectories(q_from, q_to, state_in_switch, omega_zero, drone);
    a_switches[i_sw].rot = rot_tr;
    // INFO("switchtime+rot duration" << (t_switch + rot_tr.time()))
    if (rot_tr.time() == 0) {
      ERROR_RED("should not use zero rotation!!!!");
      INFO(rot_tr)

      INFO("q_from " << q_from.coeffs().transpose());
      INFO("q_to " << q_to.coeffs().transpose());
      exit(1);
    }
    INFO("\t" << rot_tr)

    // debug part begin
    const DroneState end_state = rot_tr.get_end_state();
    Quaternion q_end = end_state.getAttitude();
    // INFO("q_end " << q_end.w() << " " << q_end.vec().transpose());
    Vector<3> t1 = q_end * Vector<3>(0, 0, 1);
    Vector<3> t2 = q_to * Vector<3>(0, 0, 1);
    INFO("\tz end " << t1.transpose() << " z start " << t2.transpose()
                    << " should equals")
    if ((t1 - t2).norm() > PRECISION) {
      INFO("bad thrust dir quaternion 2")
      INFO("t1 " << t1.transpose());
      INFO("t2 " << t2.transpose());
      exit(1);
    }
    // debug part end
  }


  // check which switches should be merged together based on their time
  std::vector<std::tuple<int, int>> merged_switches;
  for (int var1 = 1; var1 < a_switches.size(); ++var1) {
    for (int var2 = var1 + 1; var2 < a_switches.size(); ++var2) {
      const Scalar &sw1_t = a_switches[var1].time;
      const Scalar &sw2_t = a_switches[var2].time;
      const Scalar &sw1_dur = a_switches[var1].rot.time();
      const Scalar &sw2_dur = a_switches[var2].rot.time();
      const Scalar sw1_b = sw1_t - 0.5 * sw1_dur;
      const Scalar sw1_e = sw1_t + 0.5 * sw1_dur;
      const Scalar sw2_b = sw2_t + 0.5 * sw2_dur;
      const Scalar sw2_e = sw2_t + 0.5 * sw2_dur;
      if ((sw1_b <= sw2_b && sw2_b <= sw1_e) ||
          (sw1_b <= sw2_e && sw2_e <= sw1_e) ||
          (sw2_b <= sw1_b && sw1_b <= sw2_e) ||
          (sw2_b <= sw1_e && sw1_e <= sw2_e)) {
        merged_switches.push_back({var1, var2});
        INFO("need to merge " << var1 << " and " << var2);
      }
    }
  }

  // use the info what switches should be merged to create final switches list
  std::vector<aswitch> final_switches;
  const int mss = merged_switches.size();
  INFO("number merged switches " << mss)
  if (mss > 0) {
    if (mss == 1) {
      auto [sw1_id, sw2_id] = merged_switches[0];
      const DroneState start_state = a_switches[sw1_id].rot.get_start_state();
      const DroneState end_state = a_switches[sw2_id].rot.get_end_state();
      const Quaternion q_start = start_state.getAttitude();
      const Quaternion q_end = end_state.getAttitude();
      // INFO("rot tr ms1 ");
      TrRMaxAcc3D rot_tr =
        calc_rot_trajectories(q_start, q_end, start_state, omega_zero, drone);
      if (a_switches[sw1_id].time > a_switches[sw2_id].time) {
        ERROR("this should nt happen 4");
        exit(1);
      }
      aswitch sw_merg;
      sw_merg.time = a_switches[sw1_id].time;
      sw_merg.time_end = a_switches[sw2_id].time;
      sw_merg.axis = -1;
      sw_merg.q = q_end;
      sw_merg.rot = rot_tr;
      final_switches.push_back(a_switches[0]);

      bool added_single = false;
      for (int var = 1; var < a_switches.size(); ++var) {
        if (var != sw1_id && var != sw2_id) {
          added_single = true;
          if (sw_merg.time > a_switches[var].time) {
            final_switches.push_back(a_switches[var]);
            final_switches.push_back(sw_merg);
          } else {
            final_switches.push_back(sw_merg);
            final_switches.push_back(a_switches[var]);
          }
        }
      }
      if (!added_single) {
        final_switches.push_back(sw_merg);
      }

    } else if (mss == 2) {
      int sw1_id, sw2_id;
      std::tie(sw1_id, std::ignore) = merged_switches[0];
      std::tie(std::ignore, sw2_id) = merged_switches[1];
      const DroneState start_state = a_switches[sw1_id].rot.get_start_state();
      const DroneState end_state = a_switches[sw2_id].rot.get_end_state();
      const Quaternion q_start = start_state.getAttitude();
      const Quaternion q_end = end_state.getAttitude();
      // INFO("rot tr ms2 ");
      TrRMaxAcc3D rot_tr =
        calc_rot_trajectories(q_start, q_end, start_state, omega_zero, drone);

      if (a_switches[sw1_id].time > a_switches[sw2_id].time) {
        ERROR("this should nt happen 5");
        exit(1);
      }
      aswitch sw_merg;
      sw_merg.time = a_switches[sw1_id].time;
      sw_merg.time_end = a_switches[sw2_id].time;
      sw_merg.axis = -1;
      sw_merg.q = q_end;
      sw_merg.rot = rot_tr;
      final_switches.push_back(a_switches[0]);
      final_switches.push_back(sw_merg);
    } else {
      ERROR("this should not happen 6");
      exit(1);
    }
  } else {
    final_switches = a_switches;
  }

  // now we have the final switches list final_switches and the acceleration
  // profile xyz....
  // concatenate the two things

  INFO("final_switches size " << final_switches.size());

  Scalar rest_time = 0;
  TrMaxAcc3D to_cut = xyz;
  for (int i = 1; i < final_switches.size();
       ++i) {  // skip the 0 as it is the beggining
    // INFO("switch t " << final_switches[i].time << " axis "
    //                  << final_switches[i].axis << " q "
    //                  << final_switches[i].q.vec().transpose())
    INFO("rot id " << i)
    // INFO("rest " << xyz.x.t1 - rest_time);
    // INFO("switch time " << final_switches[i].time)
    TrMaxAcc3D bef, aft;
    if (final_switches[i].time == final_switches[i].time_end) {
      INFO("same time with axis " << final_switches[i].axis)
      std::tie(bef, aft) =
        to_cut.split_acc_switch_by_axis(final_switches[i].axis);
      // INFO("\t\tbef " << bef)
      // INFO("\t\taft " << aft)
    } else {
      INFO("different time")
      std::tie(bef, std::ignore) =
        to_cut.split_in_time(final_switches[i].time - rest_time);
      std::tie(std::ignore, aft) =
        to_cut.split_in_time(final_switches[i].time_end - rest_time);
    }
    profile_trans.push_back(bef);
    profile_rot.push_back(final_switches[i].rot);

    // debug only
    Vector<3> t1_bef =
      bef.get_start_state().getAttitude() * Vector<3>(0, 0, 1);  // trans i
    Vector<3> t2_bef = final_switches[i - 1].rot.get_end_state().getAttitude() *
                       Vector<3>(0, 0, 1);  // rot i-1

    // if (!t2_bef.allFinite()) {
    //   INFO("not finite z axis")
    //   INFO("bef.get_end_state()" << final_switches[i -
    //   1].rot.get_end_state()) INFO("bef.get_end_state() qx"
    //        << final_switches[i - 1].rot.get_end_state().qx)
    //   INFO("bef.get_end_state() qx norm"
    //        << final_switches[i - 1].rot.get_end_state().qx.norm())
    //   INFO(final_switches[i - 1].rot)
    //   Quaternion qtst = final_switches[i -
    //   1].rot.get_end_state().getAttitude(); INFO(qtst.w() << " " << qtst.x()
    //   << " " << qtst.y() << " " << qtst.z()) exit(1);
    // }
    INFO(" z start " << t1_bef.transpose() << "z bef end " << t2_bef.transpose()
                     << " should equals")
    if (!t1_bef.allFinite() || !t2_bef.allFinite() ||
        (t1_bef - t2_bef).norm() > PRECISION) {
      INFO("bad thrust dir quaternion 3")
      INFO("t1_bef " << t1_bef.transpose());
      INFO("t2_bef " << t2_bef.transpose());
      INFO("final_switches[i - 1].q " << final_switches[i - 1].q.w() << " "
                                      << final_switches[i - 1].q.x() << " "
                                      << final_switches[i - 1].q.y() << " "
                                      << final_switches[i - 1].q.z())
      exit(1);
    }

    Vector<3> t1 = bef.get_end_state().getAttitude() * Vector<3>(0, 0, 1);
    Vector<3> t2 = final_switches[i].rot.get_start_state().getAttitude() *
                   Vector<3>(0, 0, 1);
    INFO("z end " << t1.transpose() << " z start " << t2.transpose()
                  << " should equals")
    if (!t1.allFinite() || !t2.allFinite() || (t1 - t2).norm() > PRECISION) {
      INFO("bad thrust dir quaternion 3")
      INFO("t1 " << t1.transpose());
      INFO("t2 " << t2.transpose());

      exit(1);
    }
    // debug only

    to_cut = aft;

    rest_time = final_switches[i].time_end;
  }

  INFO("to_cut " << to_cut)
  profile_trans.push_back(to_cut);

  INFO("profile_trans size " << profile_trans.size())
  INFO("profile_rot size " << profile_rot.size())

  // test being
  Vector<3> t1_last_trans =
    profile_trans.back().get_start_state().getAttitude() * Vector<3>(0, 0, 1);
  Vector<3> t2_last_trans =
    profile_rot.back().get_end_state().getAttitude() * Vector<3>(0, 0, 1);
  INFO("z last_trans " << t1_last_trans.transpose() << " z start "
                       << t2_last_trans.transpose() << " should equals")
  if ((t1_last_trans - t2_last_trans).norm() > PRECISION) {
    INFO("bad thrust dir quaternion 4")
    INFO("t1_last_trans " << t1_last_trans.transpose());
    INFO("t2_last_trans " << t2_last_trans.transpose());
    exit(1);
  }

  // test end

  // INFO("profile_trans size " << profile_trans.size())
  // INFO("profile_rot size " << profile_rot.size())

  // const Vector<3> thrust_dir_end = -acc - GVEC;
  INFO_VAR(thrust_dir_end)
  Quaternion q_target_thrust_end =
    Quaternion::FromTwoVectors(Vector<3>(0, 0, 1), thrust_dir_end);
  const Quaternion q_to = to_state.getAttitude();

  INFO_VAR(thrust_dir_end)

  INFO("thrust_dir_end " << thrust_dir_end.normalized())

  INFO("q_to vec " << q_to * Vector<3>(0, 0, 1))


  // rotation between final thrust orientation and to_state orientation
  TrRMaxAcc3D tr_roll_pitch_end = calc_rot_trajectories(
    q_target_thrust_end, q_to, to_state, omega_zero, drone);
  profile_rot.push_back(tr_roll_pitch_end);

  INFO_VAR(profile_trans.back());
  INFO_VAR(profile_trans.back().get_end_state().getAttitude() *
           Vector<3>(0, 0, 1));

  // deubg being
  Vector<3> t1 =
    profile_trans.back().get_end_state().getAttitude() * Vector<3>(0, 0, 1);
  Vector<3> t2 =
    tr_roll_pitch_end.get_start_state().getAttitude() * Vector<3>(0, 0, 1);
  INFO("z end " << t1.transpose() << " z start " << t2.transpose()
                << " should equals")
  if ((t1 - t2).norm() > PRECISION) {
    INFO("bad thrust dir quaternion 5")
    INFO("t1 " << t1.transpose());
    INFO("t2 " << t2.transpose());
    DroneState start_tst = tr_roll_pitch_end.get_start_state();
    INFO_VAR(start_tst)
    DroneState end_tst = profile_trans.back().get_end_state();
    INFO_VAR(end_tst)
    INFO_VAR((t1 - t2).norm())
    exit(1);
  }
  // debug end

  INFO("has rotations " << profile_rot.size());
  INFO("has translations " << profile_trans.size());

  Primitive pr;
  pr.rotations = profile_rot;
  pr.translations = profile_trans;
  pr.time = pr.calc_time();

  INFO("checking after creation bef")
  check_primitive(pr);
  INFO("checking after creation aft")
  return pr;
}

std::vector<Command> MotionPrimitive::get_motor_command_rot(
  const TrRMaxAcc3D rot, const int c_id, Drone *drone) {
  // INFO("get_motor_command_rot " << c_id);
  std::vector<Command> cmds;
  if (rot.time() > 0 && rot.rotation.a > 0) {
    int id = c_id;

    Command cmd1;
    cmd1.id = id;
    cmd1.time = rot.rotation.t1;
    std::tie(std::ignore, std::ignore, cmd1.command, std::ignore) =
      drone->get_max_motors_for_rotation(rot.rotation_axis);
    cmd1.type = CommandType::MAX_TAU_POSITIVE;
    cmd1.x_y_rotation_vector = rot.rotation_axis;
    cmds.push_back(cmd1);

    // for debug only
    if (cmd1.command.maxCoeff() - drone->max_t_motor_ > PRECISION ||
        cmd1.command.minCoeff() - drone->min_t_motor_ < -PRECISION) {
      INFO_RED("too large/small motor value")
      INFO_VAR(cmd1.command.maxCoeff())
      INFO_VAR(cmd1.command.minCoeff())
      INFO_VAR(drone->max_t_motor_)
      INFO_VAR(drone->min_t_motor_)
      INFO("diff " << (cmd1.command.maxCoeff() - drone->max_t_motor_))
      INFO_VAR(cmd1.command.transpose())
      exit(1);
    }

    if (rot.rotation.t2 > 0) {
      id++;
      Command cmd2;
      cmd2.id = id;
      cmd2.time = rot.rotation.t2;
      cmd2.type = CommandType::MAX_OMEGA_ROTATION;
      cmd2.x_y_rotation_vector = rot.rotation_axis;
      //    const Scalar& theta_from,
      // const Scalar& theta_to,
      // const Vector<3>& rot_axis,
      // const Scalar& time
      cmd2.command = drone->get_const_rotation_not_fall_thrust(
        rot.q_from, rot.rotation.p1, rot.rotation.p2, rot.rotation_axis,
        rot.rotation.t2);
      cmds.push_back(cmd2);

      // for debug only
      if (cmd2.command.maxCoeff() - drone->max_t_motor_ > PRECISION ||
          cmd2.command.minCoeff() - drone->min_t_motor_ < -PRECISION) {
        INFO("too large/small motor value")
        INFO_VAR(cmd2.x_y_rotation_vector)
        INFO_VAR(rot.rotation.p1)
        INFO_VAR(rot.rotation.p2)
        INFO_VAR(rot.rotation.t2)
        INFO("rot.q_from " << rot.q_from.w() << " " << rot.q_from.x() << " "
                           << rot.q_from.y() << " " << rot.q_from.z())
        INFO_VAR(cmd2.command.transpose())
        exit(1);
      }
    } else {
      INFO_RED("omit const vel rot part")
    }


    id++;
    Command cmd3;
    cmd3.id = id;
    cmd3.time = rot.rotation.t3;
    std::tie(std::ignore, std::ignore, cmd3.command, std::ignore) =
      drone->get_max_motors_for_rotation(-rot.rotation_axis);
    cmd3.type = CommandType::MAX_TAU_NEGATIVE;
    cmd3.x_y_rotation_vector = rot.rotation_axis;
    cmds.push_back(cmd3);

    // for debug only
    if (cmd3.command.maxCoeff() - drone->max_t_motor_ > PRECISION ||
        cmd3.command.minCoeff() - drone->min_t_motor_ < -PRECISION) {
      INFO("too large/small motor value")
      INFO_VAR(cmd3.command.maxCoeff())
      INFO_VAR(drone->max_t_motor_)
      INFO_VAR(cmd3.command.transpose())
      exit(1);
    }

  } else {
    ERROR_RED("this should not happen");
    INFO_VAR(rot.rotation.a);
    INFO_VAR(rot.time());
    INFO_VAR(rot);
    // exit(1);
  }
  return cmds;
}

Command MotionPrimitive::get_motor_command_trans(const TrMaxAcc3D trans,
                                                 const int c_id,
                                                 const Drone *drone) {
  // INFO("get_motor_command_trans " << c_id);
  Command cmd;
  cmd.id = c_id;
  cmd.time = trans.time();
  cmd.type = CommandType::RISE;
  cmd.command = Vector<4>::Ones() * drone->max_t_motor_;
  cmd.x_y_rotation_vector = Vector<3>::Constant(NAN);
  return cmd;
}

std::vector<std::vector<DroneState>> MotionPrimitive::get_samples(
  Primitive primitive, std::vector<std::tuple<int, int>> &gate_primitive_sizes,
  const double desired_num_samples, const double rotation_sample_mult_ratio,
  Drone *drone) {
  // const double desired_num_samples = 1000.0;
  // const double rotation_sample_mult_ratio = 10.0;
  INFO("get_samples begin")
  INFO_VAR(primitive.rotations.size())
  INFO_VAR(primitive.translations.size())
  std::vector<std::vector<DroneState>> samples;

  double rotation_time, translation_time = 0;
  for (int var = 0; var < primitive.rotations.size(); ++var) {
    rotation_time += primitive.rotations[var].time();
  }
  for (int var = 0; var < primitive.translations.size(); ++var) {
    translation_time += primitive.translations[var].time();
  }

  const double translation_dt =
    (rotation_time * rotation_sample_mult_ratio + translation_time) /
    desired_num_samples;
  const double rotation_dt = translation_dt / rotation_sample_mult_ratio;
  INFO_VAR(translation_dt);
  INFO_VAR(rotation_dt);

  double tr_start_time = 0;
  int part_id;

  // for all gates
  for (int gatei = 0; gatei < gate_primitive_sizes.size(); ++gatei) {
    int pr_rot_from = std::get<0>(gate_primitive_sizes[gatei]);
    int pr_rot_to = std::get<1>(gate_primitive_sizes[gatei]);
    INFO_VAR(pr_rot_from)
    INFO_VAR(pr_rot_to)
    const int part_id = samples.size();
    samples.push_back(std::vector<DroneState>());

    // const int trans_size = primitive.translations.size();
    for (int var = pr_rot_from; var <= pr_rot_to; ++var) {
      const double rot_t = primitive.rotations[var].time();
      const double num_samples_rot = ceil(rot_t / rotation_dt);
      for (int si = 0; si < num_samples_rot; ++si) {
        const double time_in_tr = rot_t * (si / num_samples_rot);
        DroneState state = primitive.rotations[var].state_in_time(time_in_tr);
        state.t = tr_start_time + time_in_tr;
        samples[part_id].push_back(state);
      }
      tr_start_time += rot_t;

      if (var < pr_rot_to) {
        const double trans_t = primitive.translations[var].time();
        const double num_samples_trans = ceil(trans_t / translation_dt);
        for (int si = 0; si < num_samples_trans; ++si) {
          const double time_in_tr = trans_t * (si / num_samples_trans);
          DroneState state =
            primitive.translations[var].state_in_time(time_in_tr);
          state.t = tr_start_time + time_in_tr;
          samples[part_id].push_back(state);
        }
        tr_start_time += trans_t;
      }
    }
  }
  INFO("get_samples end")
  // exit(1);
  return samples;
}


std::vector<std::vector<DroneState>> MotionPrimitive::get_samples_plain(
  Primitive primitive, std::vector<std::tuple<int, int>> &gate_primitive_sizes,
  const double desired_dt, Drone *drone) {
  // sample only pmm
  INFO("get_samples_plain begin, desired_dt " << desired_dt)
  std::vector<std::vector<DroneState>> samples;

  double translation_time = 0;
  for (int var = 0; var < primitive.translations.size(); ++var) {
    translation_time += primitive.translations[var].time();
  }

  const double translation_dt = desired_dt;
  // translation_time / desired_num_samples;

  double tr_start_time = 0;
  int part_id = 0;

  for (int gatei = 0; gatei < gate_primitive_sizes.size(); ++gatei) {
    int pr_rot_from = std::get<0>(gate_primitive_sizes[gatei]);
    int pr_rot_to = std::get<1>(gate_primitive_sizes[gatei]);
    INFO_VAR(pr_rot_from)
    INFO_VAR(pr_rot_to)
    const int part_id = samples.size();
    samples.push_back(std::vector<DroneState>());

    INFO(" trans size " << primitive.translations.size())
    for (int var = pr_rot_from; var < pr_rot_to; ++var) {
      const double trans_t = primitive.translations[var].time();
      const double num_samples_trans = ceil(trans_t / translation_dt);
      for (int si = 0; si < num_samples_trans; ++si) {
        const double time_in_tr = trans_t * (si / num_samples_trans);
        DroneState state =
          primitive.translations[var].state_in_time(time_in_tr);
        state.t = tr_start_time + time_in_tr;
        state.command.id = part_id;
        samples[part_id].push_back(state);
      }
      tr_start_time += trans_t;
    }
  }

  INFO("get_samples end")
  // exit(1);
  return samples;
}

std::vector<Command> MotionPrimitive::get_motor_commands(
  const Primitive primitive, Drone *drone) {
  std::vector<Command> cmds;
  int c_id = 0;
  // INFO("rot " << 0);
  std::vector<Command> rot_cmds_start =
    get_motor_command_rot(primitive.rotations[0], c_id, drone);
  c_id += rot_cmds_start.size();
  cmds.insert(cmds.end(), rot_cmds_start.begin(), rot_cmds_start.end());
  for (int var = 0; var < primitive.translations.size(); ++var) {
    // INFO(primitive.translations[var])
    cmds.push_back(
      get_motor_command_trans(primitive.translations[var], c_id, drone));
    c_id++;
    // INFO("rot " << var + 1);

    std::vector<Command> rot_cmds =
      get_motor_command_rot(primitive.rotations[var + 1], c_id, drone);

    c_id += rot_cmds.size();
    cmds.insert(cmds.end(), rot_cmds.begin(), rot_cmds.end());
    INFO("after prim id " << (var + 1) << " cmd " << cmds.size());
  }
  // exit(1);
  return cmds;
}


TrRMaxAcc3D MotionPrimitive::calc_rot_trajectories(
  const Quaternion &q_from, const Quaternion &q_to,
  const DroneState &state_in_switch, const Vector<3> omega_after,
  const Drone *drone) {
  // INFO("calc_rot_trajectories begin");
  // INFO("q_from " << q_from.w() << " " << q_from.vec().transpose());
  // INFO("q_to " << q_to.w() << " " << q_to.vec().transpose());


  // Quaternion new_q_to, q_xy;
  const auto [new_q_to, q_xy] = decompose_xy_z(q_from, q_to);
  // INFO("new_q_to " << new_q_to.w() << " " << new_q_to.vec().transpose());
  // INFO("q_xy " << q_xy.w() << " " << q_xy.vec().transpose());

  const AngleAxis ang_ax(q_xy);
  const Vector<3> rotation_axis = ang_ax.axis();
  // INFO("rotation_axis " << rotation_axis.transpose());
  Scalar ang_acc_rot_vec, max_omega_rot_vec;
  std::tie(ang_acc_rot_vec, max_omega_rot_vec, std::ignore, std::ignore) =
    drone->get_max_motors_for_rotation(rotation_axis);

  // INFO_VAR(ang_acc_rot_vec);
  // INFO_VAR(max_omega_rot_vec);
  /*
    TrRMaxAcc3D calc_pitch_roll(const Quaternion &q_from, const Quaternion
    &q_to, const Vector<3> from_omegas, const Vector<3> to_omegas, const
    Vector<3> from_state_p, const Vector<3> from_state_v, const Drone *drone,
                                const double rot_acc, const double rot_vel)
                                */
  const Vector<3> from_state_p =
    state_in_switch.x.segment<DroneState::IDX::NPOS>(DroneState::IDX::POS);
  const Vector<3> from_state_v =
    state_in_switch.x.segment<DroneState::IDX::NVEL>(DroneState::IDX::VEL);
  const Vector<3> from_omegas =
    state_in_switch.x.segment<DroneState::IDX::NOME>(DroneState::IDX::OME);

  TrRMaxAcc3D to_return =
    calc_pitch_roll(q_from, new_q_to, from_omegas, omega_after, from_state_p,
                    from_state_v, drone, ang_acc_rot_vec, max_omega_rot_vec);

  return to_return;
}

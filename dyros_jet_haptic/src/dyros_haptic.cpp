
#include "dyros_jet_haptic/dyros_jet_haptic.h"

DyrosHaptic::DyrosHaptic()
{
    quit_flag_ = false;
    end_effector_ = true;
    command_frame_ = false;
    scale_ = 10.0;
    haptic_button_ = 0;
    haptic_button_pre_ = 0;
    haptic_pos_x_ = 0.0;
    haptic_pos_y_ = 0.0;
    haptic_pos_z_ = 0.0;
    haptic_pos_x_pre_ = 0.0;
    haptic_pos_y_pre_ = 0.0;
    haptic_pos_z_pre_ = 0.0;
    haptic_ang_x_ = 0.0;
    haptic_ang_y_ = 0.0;
    haptic_ang_z_ = 0.0;
    vx_, vy_, vz_, wx_, wy_, wz_, vg_, fx_, fy_, fz_, tx_, ty_, tz_, fg_ = 0.0;

    haptic_publisher_.init(nh_,"/dyros_jet/haptic_command",5);
}


void DyrosHaptic::hapticLoop() {
    ros::Rate r(200);

    int major, minor, release, revision;
    dhdGetSDKVersion(&major, &minor, &release, &revision);
    printf("\n");
    printf("Force Dimension - %d.%d.%d.%d\n",major,minor,release,revision);
    printf("(C) 2001-2015 Force Dimension\n");
    printf("All Rights Reserved.\n\n");

    dhdEnableExpertMode();

    if(dhdOpen()<0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep (2.0);
        return;
    }
    dhdEnableForce(DHD_ON);

    printf("press 'r' to control right hand (default)\n");
    printf("press 'l' to control left hand\n");
    printf("press 'b' to control through base frame\n");
    printf("press 'e' to control through end effector frame\n");
    printf("press 's' to change scale\n\n");
    printf("press 'q' to quit\n\n");
    printf("Haptic Information:\n\n");
    printf(" posX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
    printf("----------------------------------------------------------------------\n");

    while(!quit_flag_ && ros::ok()) {

        dhdGetLinearVelocity (&vx_, &vy_, &vz_);
        fx_ = -LINEAR_VISCOSITY * vx_;
        fy_ = -LINEAR_VISCOSITY * vy_;
        fz_ = -LINEAR_VISCOSITY * vz_;

        dhdGetAngularVelocityRad (&wx_, &wy_, &wz_);
        tx_ = -ANGULAR_VISCOSITY * wx_;
        ty_ = -ANGULAR_VISCOSITY * wy_;
        tz_ = -ANGULAR_VISCOSITY * wz_;

        dhdGetGripperLinearVelocity (&vg_);
        fg_ = -LINEAR_VISCOSITY * vg_;

        if (dhdSetForceAndTorqueAndGripperForce (fx_, fy_, fz_, tx_, ty_, tz_, fg_) < DHD_NO_ERROR) {
          printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
          quit_flag_ = 1;
        }

        haptic_button_ = dhdGetButtonMask();

        if(dhdGetPosition(&haptic_pos_x_, &haptic_pos_y_, &haptic_pos_z_) < DHD_NO_ERROR)
        {
            printf("error: cannot get position (%s)\n", dhdErrorGetLastStr());
        }
        else
        {
            printf(" %+2.4f | %+2.4f | %+2.4f | ", haptic_pos_x_ ,  haptic_pos_y_ , haptic_pos_z_ );
        }

        if(dhdGetOrientationRad(&haptic_ang_x_, &haptic_ang_y_, &haptic_ang_z_) < DHD_NO_ERROR)
        {
            printf("error: cannot get orientation (%s)\n", dhdErrorGetLastStr());
        }
        else
        {
            printf("%+2.4f | %+2.4f | %+2.4f |", haptic_ang_x_ , haptic_ang_y_ , haptic_ang_z_ );
        }

        if(haptic_button_)
        {
            printf("    Y");
        }
        else
        {
            printf("    N");
        }
        printf("       \r");


        // initialize position of haptic device
//        if(haptic_button_== 1 && haptic_button_pre_ == 0) {
//            haptic_pos_x_pre_ = haptic_pos_x_;
//            haptic_pos_y_pre_ = haptic_pos_y_;
//            haptic_pos_z_pre_ = haptic_pos_z_;
//        }


        for(int i = 0; i < 4; i++)
        {
            task_cmd_msg_.end_effector[i] = false;

            task_cmd_msg_.pose[i].position.x = 0.0;
            task_cmd_msg_.pose[i].position.y = 0.0;
            task_cmd_msg_.pose[i].position.z = 0.0;

            task_cmd_msg_.pose[i].orientation.x = 0.0;
            task_cmd_msg_.pose[i].orientation.y = 0.0;
            task_cmd_msg_.pose[i].orientation.z = 0.0;
            task_cmd_msg_.pose[i].orientation.w = 0.0;

            task_cmd_msg_.duration[i] = 0.0;
        }

        // left end effector command
        if(end_effector_ == false)
        {
            task_cmd_msg_.end_effector[2] = true;
            task_cmd_msg_.pose[2].position.x = -scale_*(haptic_pos_x_-haptic_pos_x_pre_);
            task_cmd_msg_.pose[2].position.y = -scale_*(haptic_pos_y_-haptic_pos_y_pre_);
            task_cmd_msg_.pose[2].position.z = scale_*(haptic_pos_z_-haptic_pos_z_pre_);
            task_cmd_msg_.pose[2].orientation.x =0;
            task_cmd_msg_.pose[2].orientation.y = 0;
            task_cmd_msg_.pose[2].orientation.z = 0;
            task_cmd_msg_.pose[2].orientation.w = 0;

            task_cmd_msg_.duration[2] = 0.005;

            if(!command_frame_)
                task_cmd_msg_.mode[2] = 1;
            else
                task_cmd_msg_.mode[2] = 0;
        }

         // right end effector command
        if(end_effector_ == true)
        {
            task_cmd_msg_.end_effector[3] = true;
            task_cmd_msg_.pose[3].position.x = -scale_*(haptic_pos_x_-haptic_pos_x_pre_);
            task_cmd_msg_.pose[3].position.y = -scale_*(haptic_pos_y_-haptic_pos_y_pre_);
            task_cmd_msg_.pose[3].position.z = scale_*(haptic_pos_z_-haptic_pos_z_pre_);
            task_cmd_msg_.pose[3].orientation.x = 0.0;
            task_cmd_msg_.pose[3].orientation.y = 0.0;
            task_cmd_msg_.pose[3].orientation.z = 0.0;
            task_cmd_msg_.pose[3].orientation.w = 0.0;
            task_cmd_msg_.duration[3] = 0.005;

            if(!command_frame_)
                task_cmd_msg_.mode[3] = 1;
            else
                task_cmd_msg_.mode[3] = 0;
        }

        // trick: To let the controller know that haptic is positive edge triggered if msg.duration= 0.0
        if(haptic_button_== 1 && haptic_button_pre_ == 0 || end_effector_changed_) {
            if(!end_effector_)
                task_cmd_msg_.duration[2] = 1.0;
            else
                task_cmd_msg_.duration[3] = 1.0;
           end_effector_changed_ = false;
        }

        // publish only when the button is pushed
        if(haptic_button_==1)
        {
            haptic_publisher_.msg_ = task_cmd_msg_;
            haptic_publisher_.unlockAndPublish();
        }


        haptic_pos_x_pre_ = haptic_pos_x_;
        haptic_pos_y_pre_ = haptic_pos_y_;
        haptic_pos_z_pre_ = haptic_pos_z_;
        haptic_button_pre_ = haptic_button_;


        // make exit condition
        if(dhdKbHit())
        {
            switch(dhdKbGet())
            {
            case 'q':
                quit_flag_ = true;
                break;
            case 'l':
                end_effector_ = false;
                end_effector_changed_ = true;
                printf("\n\nEnd Effector: Left arm\n");
                printf("\nposX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
                printf("----------------------------------------------------------------------\n");
                break;
            case 'r':
                end_effector_ = true;
                end_effector_changed_ = true;
                printf("\n\nEnd Effector: Right arm\n");
                printf("\nposX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
                printf("----------------------------------------------------------------------\n");
                break;
            case 's':
                printf("\n\nEnter scale parameter(default 10): \n");
                std::cin>>scale_;
                printf("\nposX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
                printf("----------------------------------------------------------------------\n");
                break;
            case 'b':
                command_frame_ = 0;
                printf("\n\nCommand through base frame! \n");
                printf("\nposX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
                printf("----------------------------------------------------------------------\n");
                break;
            case 'e':
                command_frame_ = 1;
                printf("\n\nCommand through end effector frame! \n");
                printf("\nposX[m] | posY[m] | posZ[m] |angX[rad]|angY[rad]|angZ[rad]|  button\n");
                printf("----------------------------------------------------------------------\n");
                break;
            }
        }
        ros::spinOnce();
        r.sleep();
    }


}

int main(int argc, char** argv) {

    ros::init(argc,argv,"dyros_jet_haptic");
    DyrosHaptic dyros_haptic;
    dyros_haptic.hapticLoop();
    dhdClose();
    printf("\nExit Haptic Controller\n\n");

    return 0;
}

; Auto-generated. Do not edit!


(cl:in-package cob_hwmonitor-msg)


;//! \htmlinclude hw_msg.msg.html

(cl:defclass <hw_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (temp_1_curr
    :reader temp_1_curr
    :initarg :temp_1_curr
    :type cl:fixnum
    :initform 0)
   (temp_1_min
    :reader temp_1_min
    :initarg :temp_1_min
    :type cl:fixnum
    :initform 0)
   (temp_1_max
    :reader temp_1_max
    :initarg :temp_1_max
    :type cl:fixnum
    :initform 0)
   (temp_2_curr
    :reader temp_2_curr
    :initarg :temp_2_curr
    :type cl:fixnum
    :initform 0)
   (temp_2_min
    :reader temp_2_min
    :initarg :temp_2_min
    :type cl:fixnum
    :initform 0)
   (temp_2_max
    :reader temp_2_max
    :initarg :temp_2_max
    :type cl:fixnum
    :initform 0)
   (temp_3_curr
    :reader temp_3_curr
    :initarg :temp_3_curr
    :type cl:fixnum
    :initform 0)
   (temp_3_min
    :reader temp_3_min
    :initarg :temp_3_min
    :type cl:fixnum
    :initform 0)
   (temp_3_max
    :reader temp_3_max
    :initarg :temp_3_max
    :type cl:fixnum
    :initform 0)
   (temp_4_curr
    :reader temp_4_curr
    :initarg :temp_4_curr
    :type cl:fixnum
    :initform 0)
   (temp_4_min
    :reader temp_4_min
    :initarg :temp_4_min
    :type cl:fixnum
    :initform 0)
   (temp_4_max
    :reader temp_4_max
    :initarg :temp_4_max
    :type cl:fixnum
    :initform 0)
   (temp_5_curr
    :reader temp_5_curr
    :initarg :temp_5_curr
    :type cl:fixnum
    :initform 0)
   (temp_5_min
    :reader temp_5_min
    :initarg :temp_5_min
    :type cl:fixnum
    :initform 0)
   (temp_5_max
    :reader temp_5_max
    :initarg :temp_5_max
    :type cl:fixnum
    :initform 0)
   (temp_6_curr
    :reader temp_6_curr
    :initarg :temp_6_curr
    :type cl:fixnum
    :initform 0)
   (temp_6_min
    :reader temp_6_min
    :initarg :temp_6_min
    :type cl:fixnum
    :initform 0)
   (temp_6_max
    :reader temp_6_max
    :initarg :temp_6_max
    :type cl:fixnum
    :initform 0)
   (akku_voltage_curr
    :reader akku_voltage_curr
    :initarg :akku_voltage_curr
    :type cl:fixnum
    :initform 0)
   (akku_voltage_min
    :reader akku_voltage_min
    :initarg :akku_voltage_min
    :type cl:fixnum
    :initform 0)
   (akku_voltage_max
    :reader akku_voltage_max
    :initarg :akku_voltage_max
    :type cl:fixnum
    :initform 0)
   (hals_motor_voltage_curr
    :reader hals_motor_voltage_curr
    :initarg :hals_motor_voltage_curr
    :type cl:fixnum
    :initform 0)
   (hals_motor_voltage_min
    :reader hals_motor_voltage_min
    :initarg :hals_motor_voltage_min
    :type cl:fixnum
    :initform 0)
   (hals_motor_voltage_max
    :reader hals_motor_voltage_max
    :initarg :hals_motor_voltage_max
    :type cl:fixnum
    :initform 0)
   (hals_logik_voltage_curr
    :reader hals_logik_voltage_curr
    :initarg :hals_logik_voltage_curr
    :type cl:fixnum
    :initform 0)
   (hals_logik_voltage_min
    :reader hals_logik_voltage_min
    :initarg :hals_logik_voltage_min
    :type cl:fixnum
    :initform 0)
   (hals_logik_voltage_max
    :reader hals_logik_voltage_max
    :initarg :hals_logik_voltage_max
    :type cl:fixnum
    :initform 0)
   (tablett_logik_voltage_curr
    :reader tablett_logik_voltage_curr
    :initarg :tablett_logik_voltage_curr
    :type cl:fixnum
    :initform 0)
   (tablett_logik_voltage_min
    :reader tablett_logik_voltage_min
    :initarg :tablett_logik_voltage_min
    :type cl:fixnum
    :initform 0)
   (tablett_logik_voltage_max
    :reader tablett_logik_voltage_max
    :initarg :tablett_logik_voltage_max
    :type cl:fixnum
    :initform 0)
   (arm_logik_voltage_curr
    :reader arm_logik_voltage_curr
    :initarg :arm_logik_voltage_curr
    :type cl:fixnum
    :initform 0)
   (arm_logik_voltage_min
    :reader arm_logik_voltage_min
    :initarg :arm_logik_voltage_min
    :type cl:fixnum
    :initform 0)
   (arm_logik_voltage_max
    :reader arm_logik_voltage_max
    :initarg :arm_logik_voltage_max
    :type cl:fixnum
    :initform 0)
   (tablett_motor_voltage_curr
    :reader tablett_motor_voltage_curr
    :initarg :tablett_motor_voltage_curr
    :type cl:fixnum
    :initform 0)
   (tablett_motor_voltage_min
    :reader tablett_motor_voltage_min
    :initarg :tablett_motor_voltage_min
    :type cl:fixnum
    :initform 0)
   (tablett_motor_voltage_max
    :reader tablett_motor_voltage_max
    :initarg :tablett_motor_voltage_max
    :type cl:fixnum
    :initform 0)
   (hals_motor_current_curr
    :reader hals_motor_current_curr
    :initarg :hals_motor_current_curr
    :type cl:fixnum
    :initform 0)
   (hals_motor_current_min
    :reader hals_motor_current_min
    :initarg :hals_motor_current_min
    :type cl:fixnum
    :initform 0)
   (hals_motor_current_max
    :reader hals_motor_current_max
    :initarg :hals_motor_current_max
    :type cl:fixnum
    :initform 0)
   (hals_logik_current_curr
    :reader hals_logik_current_curr
    :initarg :hals_logik_current_curr
    :type cl:fixnum
    :initform 0)
   (hals_logik_current_min
    :reader hals_logik_current_min
    :initarg :hals_logik_current_min
    :type cl:fixnum
    :initform 0)
   (hals_logik_current_max
    :reader hals_logik_current_max
    :initarg :hals_logik_current_max
    :type cl:fixnum
    :initform 0)
   (tablett_logik_current_curr
    :reader tablett_logik_current_curr
    :initarg :tablett_logik_current_curr
    :type cl:fixnum
    :initform 0)
   (tablett_logik_current_min
    :reader tablett_logik_current_min
    :initarg :tablett_logik_current_min
    :type cl:fixnum
    :initform 0)
   (tablett_logik_current_max
    :reader tablett_logik_current_max
    :initarg :tablett_logik_current_max
    :type cl:fixnum
    :initform 0)
   (arm_logik_current_curr
    :reader arm_logik_current_curr
    :initarg :arm_logik_current_curr
    :type cl:fixnum
    :initform 0)
   (arm_logik_current_min
    :reader arm_logik_current_min
    :initarg :arm_logik_current_min
    :type cl:fixnum
    :initform 0)
   (arm_logik_current_max
    :reader arm_logik_current_max
    :initarg :arm_logik_current_max
    :type cl:fixnum
    :initform 0)
   (tablett_motor_current_curr
    :reader tablett_motor_current_curr
    :initarg :tablett_motor_current_curr
    :type cl:fixnum
    :initform 0)
   (tablett_motor_current_min
    :reader tablett_motor_current_min
    :initarg :tablett_motor_current_min
    :type cl:fixnum
    :initform 0)
   (tablett_motor_current_max
    :reader tablett_motor_current_max
    :initarg :tablett_motor_current_max
    :type cl:fixnum
    :initform 0))
)

(cl:defclass hw_msg (<hw_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hw_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hw_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_hwmonitor-msg:<hw_msg> is deprecated: use cob_hwmonitor-msg:hw_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:header-val is deprecated.  Use cob_hwmonitor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'temp_1_curr-val :lambda-list '(m))
(cl:defmethod temp_1_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_1_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_1_curr instead.")
  (temp_1_curr m))

(cl:ensure-generic-function 'temp_1_min-val :lambda-list '(m))
(cl:defmethod temp_1_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_1_min-val is deprecated.  Use cob_hwmonitor-msg:temp_1_min instead.")
  (temp_1_min m))

(cl:ensure-generic-function 'temp_1_max-val :lambda-list '(m))
(cl:defmethod temp_1_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_1_max-val is deprecated.  Use cob_hwmonitor-msg:temp_1_max instead.")
  (temp_1_max m))

(cl:ensure-generic-function 'temp_2_curr-val :lambda-list '(m))
(cl:defmethod temp_2_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_2_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_2_curr instead.")
  (temp_2_curr m))

(cl:ensure-generic-function 'temp_2_min-val :lambda-list '(m))
(cl:defmethod temp_2_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_2_min-val is deprecated.  Use cob_hwmonitor-msg:temp_2_min instead.")
  (temp_2_min m))

(cl:ensure-generic-function 'temp_2_max-val :lambda-list '(m))
(cl:defmethod temp_2_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_2_max-val is deprecated.  Use cob_hwmonitor-msg:temp_2_max instead.")
  (temp_2_max m))

(cl:ensure-generic-function 'temp_3_curr-val :lambda-list '(m))
(cl:defmethod temp_3_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_3_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_3_curr instead.")
  (temp_3_curr m))

(cl:ensure-generic-function 'temp_3_min-val :lambda-list '(m))
(cl:defmethod temp_3_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_3_min-val is deprecated.  Use cob_hwmonitor-msg:temp_3_min instead.")
  (temp_3_min m))

(cl:ensure-generic-function 'temp_3_max-val :lambda-list '(m))
(cl:defmethod temp_3_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_3_max-val is deprecated.  Use cob_hwmonitor-msg:temp_3_max instead.")
  (temp_3_max m))

(cl:ensure-generic-function 'temp_4_curr-val :lambda-list '(m))
(cl:defmethod temp_4_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_4_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_4_curr instead.")
  (temp_4_curr m))

(cl:ensure-generic-function 'temp_4_min-val :lambda-list '(m))
(cl:defmethod temp_4_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_4_min-val is deprecated.  Use cob_hwmonitor-msg:temp_4_min instead.")
  (temp_4_min m))

(cl:ensure-generic-function 'temp_4_max-val :lambda-list '(m))
(cl:defmethod temp_4_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_4_max-val is deprecated.  Use cob_hwmonitor-msg:temp_4_max instead.")
  (temp_4_max m))

(cl:ensure-generic-function 'temp_5_curr-val :lambda-list '(m))
(cl:defmethod temp_5_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_5_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_5_curr instead.")
  (temp_5_curr m))

(cl:ensure-generic-function 'temp_5_min-val :lambda-list '(m))
(cl:defmethod temp_5_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_5_min-val is deprecated.  Use cob_hwmonitor-msg:temp_5_min instead.")
  (temp_5_min m))

(cl:ensure-generic-function 'temp_5_max-val :lambda-list '(m))
(cl:defmethod temp_5_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_5_max-val is deprecated.  Use cob_hwmonitor-msg:temp_5_max instead.")
  (temp_5_max m))

(cl:ensure-generic-function 'temp_6_curr-val :lambda-list '(m))
(cl:defmethod temp_6_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_6_curr-val is deprecated.  Use cob_hwmonitor-msg:temp_6_curr instead.")
  (temp_6_curr m))

(cl:ensure-generic-function 'temp_6_min-val :lambda-list '(m))
(cl:defmethod temp_6_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_6_min-val is deprecated.  Use cob_hwmonitor-msg:temp_6_min instead.")
  (temp_6_min m))

(cl:ensure-generic-function 'temp_6_max-val :lambda-list '(m))
(cl:defmethod temp_6_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:temp_6_max-val is deprecated.  Use cob_hwmonitor-msg:temp_6_max instead.")
  (temp_6_max m))

(cl:ensure-generic-function 'akku_voltage_curr-val :lambda-list '(m))
(cl:defmethod akku_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:akku_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:akku_voltage_curr instead.")
  (akku_voltage_curr m))

(cl:ensure-generic-function 'akku_voltage_min-val :lambda-list '(m))
(cl:defmethod akku_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:akku_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:akku_voltage_min instead.")
  (akku_voltage_min m))

(cl:ensure-generic-function 'akku_voltage_max-val :lambda-list '(m))
(cl:defmethod akku_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:akku_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:akku_voltage_max instead.")
  (akku_voltage_max m))

(cl:ensure-generic-function 'hals_motor_voltage_curr-val :lambda-list '(m))
(cl:defmethod hals_motor_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_voltage_curr instead.")
  (hals_motor_voltage_curr m))

(cl:ensure-generic-function 'hals_motor_voltage_min-val :lambda-list '(m))
(cl:defmethod hals_motor_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_voltage_min instead.")
  (hals_motor_voltage_min m))

(cl:ensure-generic-function 'hals_motor_voltage_max-val :lambda-list '(m))
(cl:defmethod hals_motor_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_voltage_max instead.")
  (hals_motor_voltage_max m))

(cl:ensure-generic-function 'hals_logik_voltage_curr-val :lambda-list '(m))
(cl:defmethod hals_logik_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_voltage_curr instead.")
  (hals_logik_voltage_curr m))

(cl:ensure-generic-function 'hals_logik_voltage_min-val :lambda-list '(m))
(cl:defmethod hals_logik_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_voltage_min instead.")
  (hals_logik_voltage_min m))

(cl:ensure-generic-function 'hals_logik_voltage_max-val :lambda-list '(m))
(cl:defmethod hals_logik_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_voltage_max instead.")
  (hals_logik_voltage_max m))

(cl:ensure-generic-function 'tablett_logik_voltage_curr-val :lambda-list '(m))
(cl:defmethod tablett_logik_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_voltage_curr instead.")
  (tablett_logik_voltage_curr m))

(cl:ensure-generic-function 'tablett_logik_voltage_min-val :lambda-list '(m))
(cl:defmethod tablett_logik_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_voltage_min instead.")
  (tablett_logik_voltage_min m))

(cl:ensure-generic-function 'tablett_logik_voltage_max-val :lambda-list '(m))
(cl:defmethod tablett_logik_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_voltage_max instead.")
  (tablett_logik_voltage_max m))

(cl:ensure-generic-function 'arm_logik_voltage_curr-val :lambda-list '(m))
(cl:defmethod arm_logik_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_voltage_curr instead.")
  (arm_logik_voltage_curr m))

(cl:ensure-generic-function 'arm_logik_voltage_min-val :lambda-list '(m))
(cl:defmethod arm_logik_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_voltage_min instead.")
  (arm_logik_voltage_min m))

(cl:ensure-generic-function 'arm_logik_voltage_max-val :lambda-list '(m))
(cl:defmethod arm_logik_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_voltage_max instead.")
  (arm_logik_voltage_max m))

(cl:ensure-generic-function 'tablett_motor_voltage_curr-val :lambda-list '(m))
(cl:defmethod tablett_motor_voltage_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_voltage_curr-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_voltage_curr instead.")
  (tablett_motor_voltage_curr m))

(cl:ensure-generic-function 'tablett_motor_voltage_min-val :lambda-list '(m))
(cl:defmethod tablett_motor_voltage_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_voltage_min-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_voltage_min instead.")
  (tablett_motor_voltage_min m))

(cl:ensure-generic-function 'tablett_motor_voltage_max-val :lambda-list '(m))
(cl:defmethod tablett_motor_voltage_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_voltage_max-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_voltage_max instead.")
  (tablett_motor_voltage_max m))

(cl:ensure-generic-function 'hals_motor_current_curr-val :lambda-list '(m))
(cl:defmethod hals_motor_current_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_current_curr-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_current_curr instead.")
  (hals_motor_current_curr m))

(cl:ensure-generic-function 'hals_motor_current_min-val :lambda-list '(m))
(cl:defmethod hals_motor_current_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_current_min-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_current_min instead.")
  (hals_motor_current_min m))

(cl:ensure-generic-function 'hals_motor_current_max-val :lambda-list '(m))
(cl:defmethod hals_motor_current_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_motor_current_max-val is deprecated.  Use cob_hwmonitor-msg:hals_motor_current_max instead.")
  (hals_motor_current_max m))

(cl:ensure-generic-function 'hals_logik_current_curr-val :lambda-list '(m))
(cl:defmethod hals_logik_current_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_current_curr-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_current_curr instead.")
  (hals_logik_current_curr m))

(cl:ensure-generic-function 'hals_logik_current_min-val :lambda-list '(m))
(cl:defmethod hals_logik_current_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_current_min-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_current_min instead.")
  (hals_logik_current_min m))

(cl:ensure-generic-function 'hals_logik_current_max-val :lambda-list '(m))
(cl:defmethod hals_logik_current_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:hals_logik_current_max-val is deprecated.  Use cob_hwmonitor-msg:hals_logik_current_max instead.")
  (hals_logik_current_max m))

(cl:ensure-generic-function 'tablett_logik_current_curr-val :lambda-list '(m))
(cl:defmethod tablett_logik_current_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_current_curr-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_current_curr instead.")
  (tablett_logik_current_curr m))

(cl:ensure-generic-function 'tablett_logik_current_min-val :lambda-list '(m))
(cl:defmethod tablett_logik_current_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_current_min-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_current_min instead.")
  (tablett_logik_current_min m))

(cl:ensure-generic-function 'tablett_logik_current_max-val :lambda-list '(m))
(cl:defmethod tablett_logik_current_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_logik_current_max-val is deprecated.  Use cob_hwmonitor-msg:tablett_logik_current_max instead.")
  (tablett_logik_current_max m))

(cl:ensure-generic-function 'arm_logik_current_curr-val :lambda-list '(m))
(cl:defmethod arm_logik_current_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_current_curr-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_current_curr instead.")
  (arm_logik_current_curr m))

(cl:ensure-generic-function 'arm_logik_current_min-val :lambda-list '(m))
(cl:defmethod arm_logik_current_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_current_min-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_current_min instead.")
  (arm_logik_current_min m))

(cl:ensure-generic-function 'arm_logik_current_max-val :lambda-list '(m))
(cl:defmethod arm_logik_current_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:arm_logik_current_max-val is deprecated.  Use cob_hwmonitor-msg:arm_logik_current_max instead.")
  (arm_logik_current_max m))

(cl:ensure-generic-function 'tablett_motor_current_curr-val :lambda-list '(m))
(cl:defmethod tablett_motor_current_curr-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_current_curr-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_current_curr instead.")
  (tablett_motor_current_curr m))

(cl:ensure-generic-function 'tablett_motor_current_min-val :lambda-list '(m))
(cl:defmethod tablett_motor_current_min-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_current_min-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_current_min instead.")
  (tablett_motor_current_min m))

(cl:ensure-generic-function 'tablett_motor_current_max-val :lambda-list '(m))
(cl:defmethod tablett_motor_current_max-val ((m <hw_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hwmonitor-msg:tablett_motor_current_max-val is deprecated.  Use cob_hwmonitor-msg:tablett_motor_current_max instead.")
  (tablett_motor_current_max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hw_msg>) ostream)
  "Serializes a message object of type '<hw_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_curr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_max)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hw_msg>) istream)
  "Deserializes a message object of type '<hw_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_1_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_2_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_3_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_4_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_5_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_6_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'akku_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_voltage_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_motor_current_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hals_logik_current_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_logik_current_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_logik_current_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_curr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tablett_motor_current_max)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hw_msg>)))
  "Returns string type for a message object of type '<hw_msg>"
  "cob_hwmonitor/hw_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hw_msg)))
  "Returns string type for a message object of type 'hw_msg"
  "cob_hwmonitor/hw_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hw_msg>)))
  "Returns md5sum for a message object of type '<hw_msg>"
  "decf8e5a69c1bdb66194b1b2788e3602")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hw_msg)))
  "Returns md5sum for a message object of type 'hw_msg"
  "decf8e5a69c1bdb66194b1b2788e3602")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hw_msg>)))
  "Returns full string definition for message of type '<hw_msg>"
  (cl:format cl:nil "Header header~%uint8 temp_1_curr~%uint8 temp_1_min~%uint8 temp_1_max~%uint8 temp_2_curr~%uint8 temp_2_min~%uint8 temp_2_max~%uint8 temp_3_curr~%uint8 temp_3_min~%uint8 temp_3_max~%uint8 temp_4_curr~%uint8 temp_4_min~%uint8 temp_4_max~%uint8 temp_5_curr~%uint8 temp_5_min~%uint8 temp_5_max~%uint8 temp_6_curr~%uint8 temp_6_min~%uint8 temp_6_max~%uint8 akku_voltage_curr~%uint8 akku_voltage_min~%uint8 akku_voltage_max~%uint8 hals_motor_voltage_curr~%uint8 hals_motor_voltage_min~%uint8 hals_motor_voltage_max~%uint8 hals_logik_voltage_curr~%uint8 hals_logik_voltage_min~%uint8 hals_logik_voltage_max~%uint8 tablett_logik_voltage_curr~%uint8 tablett_logik_voltage_min~%uint8 tablett_logik_voltage_max~%uint8 arm_logik_voltage_curr~%uint8 arm_logik_voltage_min~%uint8 arm_logik_voltage_max~%uint8 tablett_motor_voltage_curr~%uint8 tablett_motor_voltage_min~%uint8 tablett_motor_voltage_max~%uint8 hals_motor_current_curr~%uint8 hals_motor_current_min~%uint8 hals_motor_current_max~%uint8 hals_logik_current_curr~%uint8 hals_logik_current_min~%uint8 hals_logik_current_max~%uint8 tablett_logik_current_curr~%uint8 tablett_logik_current_min~%uint8 tablett_logik_current_max~%uint8 arm_logik_current_curr~%uint8 arm_logik_current_min~%uint8 arm_logik_current_max~%uint8 tablett_motor_current_curr~%uint8 tablett_motor_current_min~%uint8 tablett_motor_current_max~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hw_msg)))
  "Returns full string definition for message of type 'hw_msg"
  (cl:format cl:nil "Header header~%uint8 temp_1_curr~%uint8 temp_1_min~%uint8 temp_1_max~%uint8 temp_2_curr~%uint8 temp_2_min~%uint8 temp_2_max~%uint8 temp_3_curr~%uint8 temp_3_min~%uint8 temp_3_max~%uint8 temp_4_curr~%uint8 temp_4_min~%uint8 temp_4_max~%uint8 temp_5_curr~%uint8 temp_5_min~%uint8 temp_5_max~%uint8 temp_6_curr~%uint8 temp_6_min~%uint8 temp_6_max~%uint8 akku_voltage_curr~%uint8 akku_voltage_min~%uint8 akku_voltage_max~%uint8 hals_motor_voltage_curr~%uint8 hals_motor_voltage_min~%uint8 hals_motor_voltage_max~%uint8 hals_logik_voltage_curr~%uint8 hals_logik_voltage_min~%uint8 hals_logik_voltage_max~%uint8 tablett_logik_voltage_curr~%uint8 tablett_logik_voltage_min~%uint8 tablett_logik_voltage_max~%uint8 arm_logik_voltage_curr~%uint8 arm_logik_voltage_min~%uint8 arm_logik_voltage_max~%uint8 tablett_motor_voltage_curr~%uint8 tablett_motor_voltage_min~%uint8 tablett_motor_voltage_max~%uint8 hals_motor_current_curr~%uint8 hals_motor_current_min~%uint8 hals_motor_current_max~%uint8 hals_logik_current_curr~%uint8 hals_logik_current_min~%uint8 hals_logik_current_max~%uint8 tablett_logik_current_curr~%uint8 tablett_logik_current_min~%uint8 tablett_logik_current_max~%uint8 arm_logik_current_curr~%uint8 arm_logik_current_min~%uint8 arm_logik_current_max~%uint8 tablett_motor_current_curr~%uint8 tablett_motor_current_min~%uint8 tablett_motor_current_max~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hw_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hw_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'hw_msg
    (cl:cons ':header (header msg))
    (cl:cons ':temp_1_curr (temp_1_curr msg))
    (cl:cons ':temp_1_min (temp_1_min msg))
    (cl:cons ':temp_1_max (temp_1_max msg))
    (cl:cons ':temp_2_curr (temp_2_curr msg))
    (cl:cons ':temp_2_min (temp_2_min msg))
    (cl:cons ':temp_2_max (temp_2_max msg))
    (cl:cons ':temp_3_curr (temp_3_curr msg))
    (cl:cons ':temp_3_min (temp_3_min msg))
    (cl:cons ':temp_3_max (temp_3_max msg))
    (cl:cons ':temp_4_curr (temp_4_curr msg))
    (cl:cons ':temp_4_min (temp_4_min msg))
    (cl:cons ':temp_4_max (temp_4_max msg))
    (cl:cons ':temp_5_curr (temp_5_curr msg))
    (cl:cons ':temp_5_min (temp_5_min msg))
    (cl:cons ':temp_5_max (temp_5_max msg))
    (cl:cons ':temp_6_curr (temp_6_curr msg))
    (cl:cons ':temp_6_min (temp_6_min msg))
    (cl:cons ':temp_6_max (temp_6_max msg))
    (cl:cons ':akku_voltage_curr (akku_voltage_curr msg))
    (cl:cons ':akku_voltage_min (akku_voltage_min msg))
    (cl:cons ':akku_voltage_max (akku_voltage_max msg))
    (cl:cons ':hals_motor_voltage_curr (hals_motor_voltage_curr msg))
    (cl:cons ':hals_motor_voltage_min (hals_motor_voltage_min msg))
    (cl:cons ':hals_motor_voltage_max (hals_motor_voltage_max msg))
    (cl:cons ':hals_logik_voltage_curr (hals_logik_voltage_curr msg))
    (cl:cons ':hals_logik_voltage_min (hals_logik_voltage_min msg))
    (cl:cons ':hals_logik_voltage_max (hals_logik_voltage_max msg))
    (cl:cons ':tablett_logik_voltage_curr (tablett_logik_voltage_curr msg))
    (cl:cons ':tablett_logik_voltage_min (tablett_logik_voltage_min msg))
    (cl:cons ':tablett_logik_voltage_max (tablett_logik_voltage_max msg))
    (cl:cons ':arm_logik_voltage_curr (arm_logik_voltage_curr msg))
    (cl:cons ':arm_logik_voltage_min (arm_logik_voltage_min msg))
    (cl:cons ':arm_logik_voltage_max (arm_logik_voltage_max msg))
    (cl:cons ':tablett_motor_voltage_curr (tablett_motor_voltage_curr msg))
    (cl:cons ':tablett_motor_voltage_min (tablett_motor_voltage_min msg))
    (cl:cons ':tablett_motor_voltage_max (tablett_motor_voltage_max msg))
    (cl:cons ':hals_motor_current_curr (hals_motor_current_curr msg))
    (cl:cons ':hals_motor_current_min (hals_motor_current_min msg))
    (cl:cons ':hals_motor_current_max (hals_motor_current_max msg))
    (cl:cons ':hals_logik_current_curr (hals_logik_current_curr msg))
    (cl:cons ':hals_logik_current_min (hals_logik_current_min msg))
    (cl:cons ':hals_logik_current_max (hals_logik_current_max msg))
    (cl:cons ':tablett_logik_current_curr (tablett_logik_current_curr msg))
    (cl:cons ':tablett_logik_current_min (tablett_logik_current_min msg))
    (cl:cons ':tablett_logik_current_max (tablett_logik_current_max msg))
    (cl:cons ':arm_logik_current_curr (arm_logik_current_curr msg))
    (cl:cons ':arm_logik_current_min (arm_logik_current_min msg))
    (cl:cons ':arm_logik_current_max (arm_logik_current_max msg))
    (cl:cons ':tablett_motor_current_curr (tablett_motor_current_curr msg))
    (cl:cons ':tablett_motor_current_min (tablett_motor_current_min msg))
    (cl:cons ':tablett_motor_current_max (tablett_motor_current_max msg))
))

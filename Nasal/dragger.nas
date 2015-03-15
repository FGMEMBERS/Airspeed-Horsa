##############################################################################################
##############################################################################################
# Nasal script to handle drag robot, in case there is no other dragger
#
##############################################################################################
# Author:  Klaus Kerner
# Version: 2011-02-01
#
##############################################################################################
# Concepts:
# 1. search for allready existing dragger in the property tree                  done
# 2. if an dragger does not exist, create a new one                             done
# 3. place dragger in front of glider                                           done
# 4. run the dragger up into the sky                                            mostly done
# 5. after releasing tow delete the dragger                                     only stop

##############################################################################################
#### new properties in the property tree
# ai/models/dragger
# ai/models/dragger/id
# ai/models/dragger/callsign
# ai/models/dragger/valid
# ai/models/dragger/position/latitude-deg
# ai/models/dragger/position/longitude-deg
# ai/models/dragger/position/altitude-ft
# ai/models/dragger/orientation/true-heading-deg
# ai/models/dragger/orientation/pitch-deg
# ai/models/dragger/orientation/roll-deg
# ai/models/dragger/velocities/true-airspeed-kt
# ai/models/dragger/velocities/vertical-speed-fps
# sim/glider/dragger/robot/exist                        flag for existence of robot
# sim/glider/dragger/robot/run                          flag for triggering operation
# sim/glider/dragger/robot/id_AI
# sim/glider/dragger/robot/id_model
# sim/glider/dragger/robot/wp0lat_deg                   wp0 reference point for different legs
# sim/glider/dragger/robot/wp0lon_deg
# sim/glider/dragger/robot/wp0alt_m
# sim/glider/dragger/robot/leg_type                     storing type of leg, 0 start, 
#                                                                            1 turn, 
#                                                                            2 straight
#                                                                            3 end 
# sim/glider/dragger/robot/leg_distance_m               target distance in straight leg
# sim/glider/dragger/robot/leg_angle_deg                target turn angle in turn leg
# sim/glider/dragger/robot/turnside                     1: right turn, 0: left turn
# sim/glider/dragger/robot/leg_segment                  storing segment of leg0, 
#                                                                    -1 waiting for start, 
#                                                                    0 tauten, 
#                                                                    1 acceleration
#                                                       storing segment of leg1
#                                                                    2 roll in 
#                                                                    3 keep roll angle
#                                                                    4 roll out
# models/model[id_model]/path
# models/model[id_model]/longitude-deg-prop
# models/model[id_model]/latitude-deg-prop
# models/model[id_model]/elevation-ft-prop
# models/model[id_model]/heading-deg-prop

#### used properties from the property tree
# /orientation/heading-deg
# /sim/glider/dragger/hooked

##############################################################################################
# global variables:
  var dragrobot_timeincrement_s = 0.05;                     # timer increment

  # constants for describing dragger key properties
  var glob_min_speed_takeoff_mps  = 20;       # minimum speed for take-off of drag-robot
  var glob_max_speed_mps          = 36;       # maximum speed of drag-robot
  var glob_max_speed_lift_mps     = 3;        # maximum lift speed of drag-robot
  var glob_max_speed_tauten_mps   = 2;        # maximum speed to tauten the rope
  var glob_min_acceleration_mpss  = 0.5;      # minimum acceleration
  var glob_max_acceleration_mpss  = 3;        # maximum acceleration
  var glob_max_roll_deg           = 20;       # maximum roll angle
  var glob_max_rollrate_degs      = 5;        # maximum roll rate per second
  var glob_max_turnrate_degs      = 3;        # maximum turn rate per second at max roll angle

##############################################################################################
# check for allready available dragger
var checkDragger = func {
  
  #local variables
  var dragid = -1;                              # the allready used id
  var aiobjects = {};                           # vector to keep all ai objects
  
  aiobjects = props.globals.getNode("ai/models", 1).getChildren();  # store AI objects
  foreach ( var aimember; aiobjects ) { 
    # get data from aimember
    if ( (var c = aimember.getNode("callsign")) != nil) {
      var callsign = c.getValue();
      if ( callsign == "dragger" ) {
        dragid = aimember.getNode("id").getValue();
      } 
    }
  }
  return(dragid);
}

##############################################################################################
# get the next free id of ai/models members
var getFreeAIID = func {
  
  #local variables
  var aiid = 0;                                 # for the next unsused id
  var aiobjects = {};                           # vector to keep all ai objects
  
  aiobjects = props.globals.getNode("ai/models", 1).getChildren();  # store AI objects
  foreach ( var aimember; aiobjects ) { 
    # get data from aimember
    if ( (var c = aimember.getNode("id")) != nil) {
      var id = c.getValue();
      if ( aiid <= id ) {
        aiid = id +1;
      } 
    }
  }
  return(aiid);
}

##############################################################################################
# get the next free id of models/model members
var getFreeModelID = func {
  
  #local variables
  var modelid = 0;                                 # for the next unsused id
  var modelobjects = {};                           # vector to keep all model objects
  
  modelobjects = props.globals.getNode("models", 1).getChildren();  # store model objects
  foreach ( var member; modelobjects ) { 
    # get data from member
    if ( (var c = member.getNode("id")) != nil) {
      var id = c.getValue();
      if ( modelid <= id ) {
        modelid = id +1;
      } 
    }
  }
  return(modelid);
}

##############################################################################################
# create the drag robot in the ai property tree
var createDragRobot = func {
  
  # local variables
  var ac_pos = geo.aircraft_position();                      # get position of aircraft
  var ac_hd  = getprop("orientation/heading-deg");           # get heading of aircraft
  var dip    = ac_pos.apply_course_distance( ac_hd , 15 );   # initial dragger position, 
                                                               # 15m in front of glider
  var dipalt_m = geo.elevation(dip.lat(), dip.lon());        # height at dragger position
  var wp0_geo = geo.Coord.new();                             # current processed ai-plane
  
  # get the next free ai id and model id
  var freeAIid = getFreeAIID();
  var freeModelid = getFreeModelID();
  
  
  var dragger_ai  = props.globals.getNode("ai/models/dragger", 1);
  var dragger_mod = props.globals.getNode("models", 1);
  var dragger_sim = props.globals.getNode("sim/glider/dragger/robot", 1);
  
  dragger_sim.getNode("id_AI", 1).setIntValue(freeAIid);
  dragger_sim.getNode("id_model", 1).setIntValue(freeModelid);
  
  dragger_ai.getNode("id", 1).setIntValue(freeAIid);
  dragger_ai.getNode("callsign", 1).setValue("dragger");
  dragger_ai.getNode("valid", 1).setBoolValue(1);
  dragger_ai.getNode("position/latitude-deg", 1).setValue(dip.lat());
  dragger_ai.getNode("position/longitude-deg", 1).setValue(dip.lon());
  dragger_ai.getNode("position/altitude-ft", 1).setValue(dipalt_m * M2FT);
  dragger_ai.getNode("orientation/true-heading-deg", 1).setValue(ac_hd);
  dragger_ai.getNode("orientation/pitch-deg", 1).setValue(0);
  dragger_ai.getNode("orientation/roll-deg", 1).setValue(0);
  dragger_ai.getNode("velocities/true-airspeed-kt", 1).setValue(0);
  dragger_ai.getNode("velocities/vertical-speed-fps", 1).setValue(0);
  
  dragger_mod.model = dragger_mod.getChild("model", freeModelid, 1);
############################################################################################## Specific to airplane (better in Generic I think)
  dragger_mod.model.getNode("path", 1).setValue("Aircraft/ASK21-MI/Models/Dragger/robot.xml");
##############################################################################################
  dragger_mod.model.getNode("longitude-deg-prop", 1).setValue("ai/models/dragger/position/longitude-deg");
  dragger_mod.model.getNode("latitude-deg-prop", 1).setValue("ai/models/dragger/position/latitude-deg");
  dragger_mod.model.getNode("elevation-ft-prop", 1).setValue("ai/models/dragger/position/altitude-ft");
  dragger_mod.model.getNode("heading-deg-prop", 1).setValue("ai/models/dragger/orientation/true-heading-deg");
  dragger_mod.model.getNode("roll-deg-prop", 1).setValue("ai/models/dragger/orientation/roll-deg");
  dragger_mod.model.getNode("load", 1).remove();
  
  dragger_sim.getNode("exist", 1).setIntValue(1);
  dragger_sim.getNode("leg_type", 1).setIntValue(0);
  dragger_sim.getNode("leg_distance_m", 1).setValue(2000);
  dragger_sim.getNode("leg_angle_deg", 1).setValue(ac_hd);
  dragger_sim.getNode("leg_segment", 1).setIntValue(-1);
  
  wp0_geo = geo.Coord.set_latlon( dip.lat(), dip.lon(), dipalt_m );
  
  setprop("sim/glider/dragger/robot/wp0lat_deg", wp0_geo.lat() );
  setprop("sim/glider/dragger/robot/wp0lon_deg", wp0_geo.lon() );
  setprop("sim/glider/dragger/robot/wp0alt_m", wp0_geo.alt() );
}

##############################################################################################
# main function to initialize the drag roboter
var setupDragRobot = func {
  
  # look for allready existing ai object with callsign "dragger"
  var existingdragid = checkDragger();
  if ( existingdragid > -1 ) {                       # dragger allready exists, we can exit
    atc_msg(" existing dragger id: ", existingdragid);
  }
  else {                                             # dragger does not exist, we have to work
    # create a new ai object with callsign "dragger"
    # set initial position 
    createDragRobot();
    dragger_msg(" I will lift you up into the sky.");
  }
  
}

##############################################################################################
# run the drag robot for start leg
var leg0DragRobot = func {
  
  ############################################################################################
  # Strategy:
  # set flag for start
  # tauten the rope
  # accelerate up to minimum lift speed
  # switch to next leg
  
  var initpos_geo = geo.Coord.new();
  var dragpos_geo = geo.Coord.new();
  
  var oldspeed_mps     = 0;
  var oldlift_mps      = 0;
  var oldheading_deg   = 0;
  var newspeed_mps     = 0;
  var newlift_mps      = 0;
  var newliftdist_m    = 0;
  var newelevation_m   = 0;
  var distance_m       = 0;
  var leg_distance_m   = 0;
  var deltatime_s      = 0;
  var leg_angle_deg    = 0;
  
  var segment = getprop("sim/glider/dragger/robot/leg_segment");
  
  if ( segment == -1 ) { # hooked but not in start segment
    segment = 0; 
    setprop("sim/glider/dragger/robot/leg_segment", segment);
    dragger_msg(" start to tauten the tow");
  }
  
  deltatime_s = dragrobot_timeincrement_s;
  
  oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
  oldlift_mps      = getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M;
  oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
  leg_distance_m   = getprop("sim/glider/dragger/robot/leg_distance_m");
  
  initpos_geo.set_latlon( getprop("sim/glider/dragger/robot/wp0lat_deg"), 
                          getprop("sim/glider/dragger/robot/wp0lon_deg"), 
                          getprop("sim/glider/dragger/robot/wp0alt_m") );
  dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
                          getprop("ai/models/dragger/position/longitude-deg"), 
                          getprop("ai/models/dragger/position/altitude-ft") * FT2M);
  
  # check if tauten is done
  if ( ( dragpos_geo.direct_distance_to(initpos_geo) > 100 ) and ( segment == 0 ) ) { 
    segment = 1;
    setprop("sim/glider/dragger/robot/leg_segment", segment);
  }
  
  
  if ( segment == 0 ) { # tauten the rope
    if ( oldspeed_mps < glob_max_speed_tauten_mps ) {
      newspeed_mps = glob_min_acceleration_mpss * deltatime_s + oldspeed_mps; 
      distance_m = 0.5 * glob_min_acceleration_mpss * (deltatime_s * deltatime_s) 
                   + oldspeed_mps * deltatime_s;
    }
    else {
      newspeed_mps = oldspeed_mps;
      distance_m = oldspeed_mps * deltatime_s;
    }
  }
  
  if ( segment == 1 ) { # running up into the air
    if ( oldspeed_mps >= glob_max_speed_mps) {
      newspeed_mps = oldspeed_mps;
      distance_m = oldspeed_mps * deltatime_s;
    }
    else { 
      newspeed_mps = oldspeed_mps + glob_max_acceleration_mpss * deltatime_s;
      distance_m = oldspeed_mps * deltatime_s 
                   + 0.5 * glob_max_acceleration_mpss * deltatime_s * deltatime_s;
    }
  }
  
  dragpos_geo.apply_course_distance( oldheading_deg , distance_m );
  newelevation_m = geo.elevation( dragpos_geo.lat(), dragpos_geo.lon() );
  dragpos_geo.set_alt(newelevation_m);
  
  setprop("ai/models/dragger/position/latitude-deg", dragpos_geo.lat());
  setprop("ai/models/dragger/position/longitude-deg", dragpos_geo.lon());
  setprop("ai/models/dragger/position/altitude-ft", dragpos_geo.alt() * M2FT);
  setprop("ai/models/dragger/velocities/true-airspeed-kt", newspeed_mps * MPS2KT);
  setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
  
  
  # check for exit criteria
  if ( oldspeed_mps > glob_min_speed_takeoff_mps ) { 
    # set anchor point
    setprop("sim/glider/dragger/robot/wp0lat_deg", dragpos_geo.lat());
    setprop("sim/glider/dragger/robot/wp0lon_deg", dragpos_geo.lon());
    setprop("sim/glider/dragger/robot/wp0alt_m", dragpos_geo.alt());
    # set flags for next leg
    setprop("sim/glider/dragger/robot/leg_type", 2);          # next one is straight forward
    # set next exit criteria for straight leg
    var length_m = 100;                                              # first turn after 100m
    setprop("sim/glider/dragger/robot/leg_distance_m", length_m); 
    dragger_msg("  lift-off at ", dragpos_geo.alt(),"m height");
    dragger_msg(" going straight ahead for ", length_m, "m");
  }
}

##############################################################################################
# run the drag robot for turns
var leg1DragRobot = func {
  # turns are described by the turn angle, so the delta angle from heading at initial position
  # to heading from current position is the criteria for exit
  
  
  var initpos_geo = geo.Coord.new();
  var dragpos_geo = geo.Coord.new();
  
  var oldspeed_mps     = 0;
  var oldlift_mps      = 0;
  var oldheading_deg   = 0;
  var oldroll_deg      = 0;
  var deltatime_s      = 0;
  var distance_m       = 0;
  var newspeed_mps     = 0;
  var newlift_mps      = 0;
  var newelevation_m   = 0;
  var newroll_deg      = 0;
  var newturn_deg      = 0;
  var newheading_deg   = 0;
  
  
  var segment = getprop("sim/glider/dragger/robot/leg_segment");
  var side    = getprop("sim/glider/dragger/robot/turnside");
  var targetheading_deg = getprop("sim/glider/dragger/robot/leg_angle_deg");
  
  deltatime_s = dragrobot_timeincrement_s;
  
  oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
  oldlift_mps      = getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M;
  oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
  oldroll_deg      = getprop("ai/models/dragger/orientation/roll-deg");
  
  dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
                          getprop("ai/models/dragger/position/longitude-deg"), 
                          getprop("ai/models/dragger/position/altitude-ft") * FT2M);
  
  # calculate current roll angle for turns
  if ( side == 1 ) {          # right turns
    if ( segment == 2 ) {
      if ( oldroll_deg < glob_max_roll_deg) {
        # calculate new roll angle
        newroll_deg = oldroll_deg + deltatime_s * glob_max_rollrate_degs;
      }
      else {
        newroll_deg = oldroll_deg;
        setprop("sim/glider/dragger/robot/leg_segment", 3);
      }
    }
    
    if (segment == 3 ) {
      newroll_deg = oldroll_deg;
      # check for target turn 
      if ( (oldheading_deg > targetheading_deg) and (oldheading_deg <= (targetheading_deg+5)) ) { 
        # turn finished
        setprop("sim/glider/dragger/robot/leg_segment", 4);
      }
      # if yes, change segment type
    }
    
    if ( segment == 4 ) {
      if ( oldroll_deg > 0) {
        # calculate new roll angle
        newroll_deg = oldroll_deg - deltatime_s * glob_max_rollrate_degs;
      }
      else {                                                              # also exit criteria
        newroll_deg = 0;
        # set anchor point
        setprop("sim/glider/dragger/robot/wp0lat_deg", dragpos_geo.lat());
        setprop("sim/glider/dragger/robot/wp0lon_deg", dragpos_geo.lon());
        setprop("sim/glider/dragger/robot/wp0alt_m", dragpos_geo.alt());
        # set next leg
        setprop("sim/glider/dragger/robot/leg_segment", 2);
        setprop("sim/glider/dragger/robot/leg_type", 2);
        var length_m = 100;                                            # first turn after 100m
        setprop("sim/glider/dragger/robot/leg_distance_m", length_m); 
        dragger_msg("  right turn finished at ", dragpos_geo.alt(),"m height");
        dragger_msg(" entering straight leg for ", length_m, "m");
      }
    }
  }
  else {                 # left turns
    if ( segment == 2 ) {
      if ( oldroll_deg > -glob_max_roll_deg) {
        # calculate new roll angle
        newroll_deg = oldroll_deg - deltatime_s * glob_max_rollrate_degs;
      }
      else {
        newroll_deg = oldroll_deg;
        setprop("sim/glider/dragger/robot/leg_segment", 3);
      }
    }
    
    if (segment == 3 ) {
      newroll_deg = oldroll_deg;
      # check for target turn 
      if ( (oldheading_deg < targetheading_deg) and (oldheading_deg >= (targetheading_deg-5)) ) { 
        # turn finished
        setprop("sim/glider/dragger/robot/leg_segment", 4);
      }
      # if yes, change segment type
    }
    
    if ( segment == 4 ) {
      if ( oldroll_deg < 0) {
        # calculate new roll angle
        newroll_deg = oldroll_deg + deltatime_s * glob_max_rollrate_degs;
      }
      else {                                                              # also exit criteria
        newroll_deg = 0;
        # set anchor point
        setprop("sim/glider/dragger/robot/wp0lat_deg", dragpos_geo.lat());
        setprop("sim/glider/dragger/robot/wp0lon_deg", dragpos_geo.lon());
        setprop("sim/glider/dragger/robot/wp0alt_m", dragpos_geo.alt());
        # set next leg
        setprop("sim/glider/dragger/robot/leg_segment", 2);
        setprop("sim/glider/dragger/robot/leg_type", 2);
        var length_m = 100;                                            # first turn after 100m
        setprop("sim/glider/dragger/robot/leg_distance_m", length_m); 
        dragger_msg("  left turn finished at ", dragpos_geo.alt(),"m height");
        dragger_msg(" entering straight leg for ", length_m, "m");
      }
    }
  }
  # calculate current speed
  if ( oldspeed_mps >= glob_max_speed_mps) {
    newspeed_mps = oldspeed_mps;
    distance_m = oldspeed_mps * deltatime_s;
  }
  else { 
    newspeed_mps = oldspeed_mps + glob_max_acceleration_mpss * deltatime_s;
    distance_m = oldspeed_mps * deltatime_s 
                 + 0.5 * glob_max_acceleration_mpss * deltatime_s * deltatime_s;
  }
  
  # calculate current lift
  if ( oldspeed_mps > glob_min_speed_takeoff_mps ) { 
    newlift_mps = glob_max_speed_lift_mps * (oldspeed_mps - glob_min_speed_takeoff_mps) / 
                     (glob_max_speed_mps - glob_min_speed_takeoff_mps);
    newliftdist_m = newlift_mps * deltatime_s;
  }
  else {
    newlift_mps = 0;
    newliftdist_m = 0;
  }
  
  # calculate current turn rate based on roll angle
  newturn_deg = glob_max_turnrate_degs * newroll_deg / glob_max_roll_deg * deltatime_s;
  
  # calculate new heading based on turn rate
  if ( (oldheading_deg + newturn_deg) > 360 ) { # if a rightturn exceeds 360 degree heading
    newheading_deg = oldheading_deg + newturn_deg - 360;
  }
  else {
    if ( (oldheading_deg + newturn_deg) < 0 ) { # if a leftturn exceeds 0 degree heading
      newheading_deg = oldheading_deg + newturn_deg +360;
    }
    else { # for all other headings
      newheading_deg = oldheading_deg + newturn_deg;
    }
  }
  
  # calculate new position based on new heading and distance increment
  dragpos_geo.apply_course_distance( newheading_deg , distance_m );
  newelevation_m = dragpos_geo.alt() + newliftdist_m;
  dragpos_geo.set_alt(newelevation_m);
  
  
  setprop("ai/models/dragger/position/latitude-deg",         dragpos_geo.lat());
  setprop("ai/models/dragger/position/longitude-deg",        dragpos_geo.lon());
  setprop("ai/models/dragger/position/altitude-ft",          dragpos_geo.alt() * M2FT);
  setprop("ai/models/dragger/orientation/true-heading-deg",  newheading_deg);
  setprop("ai/models/dragger/orientation/roll-deg",          newroll_deg);
  setprop("ai/models/dragger/velocities/true-airspeed-kt",   oldspeed_mps * MPS2KT);
  setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
}

##############################################################################################
# run the drag robot for straight legs
var leg2DragRobot = func {
  # straight legs are described by the length, so the delta distance from initial position
  # to current position is the criteria for exit
  
  var initpos_geo = geo.Coord.new();
  var dragpos_geo = geo.Coord.new();
  
  var oldspeed_mps     = 0;
  var oldlift_mps      = 0;
  var oldheading_deg   = 0;
  var newspeed_mps     = 0;
  var newlift_mps      = 0;
  var newliftdist_m    = 0;
  var newelevation_m   = 0;
  var distance_m       = 0;
  var leg_distance_m   = 0;
  var deltatime_s      = 0;
  var leg_angle_deg    = 0;
  
  deltatime_s = dragrobot_timeincrement_s;
  
  oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
  oldlift_mps      = getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M;
  oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
  leg_distance_m   = getprop("sim/glider/dragger/robot/leg_distance_m");
  
  initpos_geo.set_latlon( getprop("sim/glider/dragger/robot/wp0lat_deg"), 
                          getprop("sim/glider/dragger/robot/wp0lon_deg"), 
                          getprop("sim/glider/dragger/robot/wp0alt_m") );
  dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
                          getprop("ai/models/dragger/position/longitude-deg"), 
                          getprop("ai/models/dragger/position/altitude-ft") * FT2M);
  
  if ( oldspeed_mps >= glob_max_speed_mps) {
    newspeed_mps = oldspeed_mps;
    distance_m = oldspeed_mps * deltatime_s;
  }
  else { 
    newspeed_mps = oldspeed_mps + glob_max_acceleration_mpss * deltatime_s;
    distance_m = oldspeed_mps * deltatime_s 
                 + 0.5 * glob_max_acceleration_mpss * deltatime_s * deltatime_s;
  }
  
  if ( oldspeed_mps > glob_min_speed_takeoff_mps ) { 
    newlift_mps = glob_max_speed_lift_mps * (oldspeed_mps - glob_min_speed_takeoff_mps) / 
                     (glob_max_speed_mps - glob_min_speed_takeoff_mps);
    newliftdist_m = newlift_mps * deltatime_s;
  }
  else {
    newlift_mps = 0;
    newliftdist_m = 0;
  }
  
  dragpos_geo.apply_course_distance( oldheading_deg , distance_m );
  newelevation_m = dragpos_geo.alt() + newliftdist_m;
  dragpos_geo.set_alt(newelevation_m);
  
  setprop("ai/models/dragger/position/latitude-deg", dragpos_geo.lat());
  setprop("ai/models/dragger/position/longitude-deg", dragpos_geo.lon());
  setprop("ai/models/dragger/position/altitude-ft", dragpos_geo.alt() * M2FT);
  setprop("ai/models/dragger/velocities/true-airspeed-kt", newspeed_mps * MPS2KT);
  setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
  
  # exit criteria to next turn
  if ( dragpos_geo.direct_distance_to(initpos_geo) > leg_distance_m ) { 
    var turn_deg = 30 + rand() * 240;                            # turn range from 30° to 270°
    if ( (oldheading_deg + turn_deg) >= 360) {
      leg_angle_deg = oldheading_deg + turn_deg - 360;
    }
    else {
      leg_angle_deg = oldheading_deg + turn_deg;
    }
    setprop("sim/glider/dragger/robot/leg_angle_deg", leg_angle_deg);
    var side = rand();
    if (side > 0.5) {
      setprop("sim/glider/dragger/robot/turnside", 1);
      dragger_msg("  straight leg finished at ", dragpos_geo.alt(),"m height");
      dragger_msg(" entering right turn for ", turn_deg, "deg");
    }
    else {
      setprop("sim/glider/dragger/robot/turnside", 0);
      dragger_msg("  straight leg finished at ", dragpos_geo.alt(),"m height");
      dragger_msg(" entering left turn for ", turn_deg, "deg");
    }
    setprop("sim/glider/dragger/robot/leg_type", 1);
    setprop("sim/glider/dragger/robot/leg_segment", 2);
  }
  
  # exit criteria to final drop-down
  if ( dragpos_geo.alt() > 8000 ) {                # max height of 8000m the dragger can reach
    dragger_msg(" we have reached max height, bye bye");
    setprop("sim/glider/dragger/robot/leg_type", 3);
    setprop("sim/glider/dragger/robot/leg_segment", 2);
  }
}

##############################################################################################
# run the drag robot for final leg
var leg3DragRobot = func {
  dragger_msg(" turn right, I turn left" );
  
  var min_speed_takeoff_mps  = 20;
  var max_speed_mps          = 36;
  var max_speed_lift_mps     = 3;
  var min_acceleration_mpss  = 1;
  var max_acceleration_mpss  = 15;
  
  var initpos_geo = geo.Coord.new();
  var oldpos_geo  = geo.Coord.new();
  var newpos_geo  = geo.Coord.new();
  
  if ( getprop("sim/glider/dragger/robot/run") == 1 ) {
    setprop("sim/glider/dragger/robot/run", 0);
  }
}

##############################################################################################
# function to switch the drag roboter without being hooked
var startDragRobot = func {
  if ( getprop("sim/glider/dragger/robot/run" ) == 1) {
    setprop("sim/glider/dragger/robot/run", 0);
    print(" stop the drag robot");
  }
  else { 
    print(" start the drag robot");
    setprop("sim/glider/dragger/robot/run", 1);
  }
}

##############################################################################################
# triggered function to run the drag roboter
var runDragRobot = func {
  if ( getprop("sim/glider/dragger/robot/run" ) == 1) {
    var leg = -1;
    
    leg = getprop("sim/glider/dragger/robot/leg_type");
    
    if ( leg == 0 ) { 
      leg0DragRobot();
    }
    
    if ( leg == 1 ) { 
      leg1DragRobot();
    }
    
    if ( leg == 2 ) { 
      leg2DragRobot();
    }
    
    if ( leg == 3 ) { 
      leg3DragRobot();
    }
    
    settimer(runDragRobot, dragrobot_timeincrement_s);
  }
}

var pulling = setlistener("/sim/glider/dragger/robot/run", runDragRobot);


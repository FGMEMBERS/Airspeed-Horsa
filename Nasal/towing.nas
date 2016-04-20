##############################################################################################
##############################################################################################
# Nasal script to handle aerotowing, with AI-dragger
#
##############################################################################################
# Author: Klaus Kerner
# Version: 2011-02-01
#
##############################################################################################
# Concepts:
# 1. search for allready existing dragger in the property tree
# 2. if an existing dragger is too far away or no dragger is available create a new one
# 3. hook in to the dragger, that is close to the glider
# 4. lift up into the air
# 5. finish towing

# existing properties from ai branch, to handle the dragger (or the drag-robot)
# /ai/models/xyz[x]                                           the dragger that lifts me up
#  ./id                                                       the id of the ai-model
#  ./callsign                                                 the callsign of the dragger
#  ./position/latitude-deg                                    latitude of dragger
#  ./position/longitude-deg                                   longitude of dragger
#  ./position/altitude-ft                                     height of dragger
#  ./orientation/true-heading-deg                             heading
#  ./orientation/pitch-deg                                    pitch
#  ./orientation/roll-deg                                     roll
#  ./velocities/true-airspeed-kt                              speed
#
#### existing properties to get glider orientation
# /orientation/heading-deg
# /orientation/pitch-deg
# /orientation/roll-deg

#### existing proterties from jsbsim config file, that are used to handle the towing forces
# /fdm/jsbsim/fcs/dragger-cmd-norm                            created by jsbsim config file
#                                                               1: dragger engaged
#                                                               0: drager not engaged
# /fdm/jsbsim/external_reactions/dragx/magnitude              created by jsbsim config file
# /fdm/jsbsim/external_reactions/dragy/magnitude              created by jsbsim config file
# /fdm/jsbsim/external_reactions/dragz/magnitude              created by jsbsim config file

# new properties to handle the dragger
# /sim/glider/dragger/dragid                 the ID of /ai/models/xyz[x]/id
# /sim/glider/dragger/rope_length_m          length of rope, set by config file or default
# /sim/glider/dragger/nominal_towforce_lbs   nominal force at nominal distance
# /sim/glider/dragger/breaking_towforce_lbs  max. force of tow
# /sim/glider/dragger/hooked                 flag to control engaged tow
#                                              1: rope hooked in
#                                              0: rope not hooked in
# /sim/glider/dragger/robot/exist            flag to control existence of robot
#                                              1: robot exists
#                                              0: robot does not exist

##############################################################################################
##############################################################################################
# global variables in this module
var timeincrement = 0.1;                        # timer increment

##############################################################################################
##############################################################################################
# findDragger

var findDragger = func {
  
  # first check for available ai-planes
  # if ai-planes are available 
  #   store them in an array
  #   get the glider position
  #   for every ai-plane
  #     calculate the distance to the glider
  #     if the distance is lower than max. tow length
  #       get id
  #       get callsign
  #       if callsign = dragger
  #         send message
  #         set property dragid
  #         leave loop
  
  # local variables
  var aiobjects = [];                            # keeps the ai-planes from the property tree
  var dragid = 0;                                # id of dragger
  var callsign = 0;                              # callsign of dragger
  var cur = geo.Coord.new();                     # current processed ai-plane
  var lat_deg = 0;                               # latitude of current processed aiobject
  var lon_deg = 0;                               # longitude of current processed aiobject
  var alt_m = 0;                                 # altitude of current processed aiobject
  var glider = geo.Coord.new();                  # coordinates of glider
  var distance_m = 0;                            # distance to ai-plane
  
  # set rope length if not defined from settings-file
  if ( getprop("/sim/glider/dragger/rope_length_m") == nil ) {
    atc_msg("rope length not defined by plane");
    atc_msg(" use default setting of 100m");
    setprop("/sim/glider/dragger/rope_length_m", 100.0);
  }
  var towlength_m = getprop("/sim/glider/dragger/rope_length_m");
  
  aiobjects = props.globals.getNode("ai/models").getChildren(); 
  glider = geo.aircraft_position(); 
  
  foreach (var aimember; aiobjects) { 
    if ( (var c = aimember.getNode("callsign") ) != nil ) { 
      callsign = c.getValue();
      dragid = aimember.getNode("id").getValue();
      if ( callsign == "dragger" ) {
        lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
        lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
        alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
        
        cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m ); 
        distance_m = (glider.distance_to(cur)); 
        
        if ( distance_m < towlength_m ) { 
          atc_msg("callsign %s with id %s nearby in %s m", callsign, dragid, distance_m);
          setprop("/sim/glider/dragger/dragid", dragid); 
          break; 
        }
        else {
          atc_msg("callsign %s with id %s too far at %s m", callsign, dragid, distance_m);
        }
      }
    }
    else {
      atc_msg("no dragger found");
    }
  }
} # End Function findDragger

##############################################################################################
##############################################################################################
# hookDragger

var hookDragger = func {
  
  # if dragid > 0
  #  set property /fdm/jsbsim/fcs/dragger-cmd-norm
  #  level plane
  
  if ( getprop("/sim/glider/dragger/dragid") != nil ) { 
    setprop("/fdm/jsbsim/fcs/dragger-cmd-norm", 1);                # closes the hook
    setprop("/sim/glider/dragger/hooked", 1); 
    atc_msg("hook closed"); 
    setprop("/orientation/roll-deg", 0); 
    atc_msg("glider leveled"); 
  }
  else { 
    atc_msg("no dragger nearby"); 
  }
  
} # End Function hookDragger

##############################################################################################
##############################################################################################
# releaseDragger

var releaseDragger = func {
  
  # first check for dragger is pulling
  # if yes
  #   opens the hook
  #   sets the forces to zero
  #   print a message
  # if no
  #   print a message
  # exit
  
  if ( getprop ("/sim/glider/dragger/hooked") ) {
    setprop  ("/fdm/jsbsim/fcs/dragger-cmd-norm",0);                 # opens the hook
    setprop("/fdm/jsbsim/external_reactions/dragx/magnitude", 0);    # set the forces to zero
    setprop("/fdm/jsbsim/external_reactions/dragy/magnitude", 0);    # set the forces to zero
    setprop("/fdm/jsbsim/external_reactions/dragz/magnitude", 0);    # set the forces to zero
    setprop("/sim/glider/dragger/hooked",0);                         # dragger is not pulling
    if ( getprop("sim/glider/dragger/robot/exist") == 1 ) {
      setprop("sim/glider/dragger/robot/run", 0);
      print("release from drag robot");
    }
    atc_msg("Hook opened, tow released");
  }
  else {                                                       # failure: winch not working
    atc_msg("Hook already opened");
  }
  
} # End Function releaseDragger

##############################################################################################
##############################################################################################
# let the dragger pull the plane up into the sky

var runDragger = func {
  
  # strategy:
  # get current positions and orientations of glider and dragger
  # calculate the forces with respect of distance and spring-coefficient of tow
  # calculate force distribution in main axes
  # do this as long as the tow is engaged at the glider
  
  # local constants describing tow properties
  var tl0 = 0.15;                      # relative length below no forces exist
  var tf0 = 0;                         # coresponding force
  # local variables
  var forcex = 0;                      # the force in x-direction, body ref system
  var forcey = 0;                      # the force in y-direction, body ref system
  var forcez = 0;                      # the force in z-direction, body ref system
  var glider = geo.Coord.new();        # keeps the glider position
  var gliderhead = 0;                  # keeps the glider heading
  var gliderpitch = 0;                 # keeps the glider pitch
  var gliderroll = 0;                  # keeps the glider roll
  var dragger = geo.Coord.new();       # keeps the dragger position
  var drlat = 0;                       # temporary latitude of dragger
  var drlon = 0;                       # temporary longitude of dragger
  var dralt = 0;                       # temporary altitude of dragger
  var dragheadto = 0;                  # heading to dragger
  var aiobjects = [];                     # keeps the ai-planes from the property tree
  var distance = 0;                    # distance glider to dragger
  var distancepr = 0;                  # projected distance glider to dragger
  var reldistance = 0;                 # relative distance glider to dragger
  var dragid = 0;                      # id of dragger
  var planeid = 0;                     # id of current processed plane
  
  # set nominal tow force if not defined from settings-file
  if ( getprop("/sim/glider/dragger/nominal_towforce_lbs") == nil ) {
    atc_msg("nominal tow force not defined by plane, use default setting of 1000lbs");
    setprop("/sim/glider/dragger/nominal_towforce_lbs", 1000.0);
  }
  var nominaltowforce = getprop("/sim/glider/dragger/nominal_towforce_lbs");
  
  # set max tow force if not defined from settings-file
  if ( getprop("/sim/glider/dragger/breaking_towforce_lbs") == nil ) {
    atc_msg("breaking tow force not defined by plane, use default setting of 2000lbs");
    setprop("/sim/glider/dragger/breaking_towforce_lbs", 2000.0);
  }
  var breakingtowforce = getprop("/sim/glider/dragger/breaking_towforce_lbs");
  
  # set tow length if not defined from settings-file
  if ( getprop("/sim/glider/dragger/rope_length_m") == nil ) {
    atc_msg("tow length not defined by plane, use default setting of 100m");
    setprop("/sim/glider/dragger/rope_length_m", 100.0);
  }
  var towlength_m = getprop("/sim/glider/dragger/rope_length_m");
  
  # do all the stuff
  
  
  if ( getprop("/sim/glider/dragger/hooked") == 1 ) {                   # is a dragger engaged
    
    if (getprop("/sim/glider/dragger/robot/exist") == 1 ) {     # store the robots in an array
      aiobjects = props.globals.getNode("ai/models").getChildren("dragger");  
    }
    else {                                                      # store the planes in an array
      aiobjects = props.globals.getNode("ai/models").getChildren("aircraft"); 
    }
    
    glider = geo.aircraft_position();                                # current glider position
    gliderpitch = getprop("/orientation/pitch-deg");
    gliderroll = getprop("/orientation/roll-deg");
    gliderhead = getprop("/orientation/heading-deg");
    
    dragid = getprop("/sim/glider/dragger/dragid");               # id of former found dragger
    
    foreach (var aimember; aiobjects) {                                  #go through the array
      id = aimember.getNode("id").getValue(); 
      if ( id == dragid ) { 
        drlat = aimember.getNode("position/latitude-deg").getValue(); 
        drlon = aimember.getNode("position/longitude-deg").getValue(); 
        dralt = (aimember.getNode("position/altitude-ft").getValue()) * FT2M; 
      }
    }
    dragger = geo.Coord.set_latlon( drlat, drlon, dralt );         # position of current plane
    
    distance = (glider.direct_distance_to(dragger));              # distance to plane in meter
    distancepr = (glider.distance_to(dragger));
    dragheadto = (glider.course_to(dragger));
    reldistance = distance / towlength_m;
    
    if ( reldistance < tl0 ) {
      forcetow = tf0;
    }
    else {
      # calculate tow force by multiplying the nominal force with reldistance powered 4
      # this gives a smooth pulling for start and a high force when the tow is nearly
      # completely taut
      forcetow = reldistance * reldistance * reldistance * reldistance * nominaltowforce;
    }
    
    if ( forcetow < breakingtowforce ) {
      
      # correct a failure, if the projected length is larger than direct length
      if (distancepr > distance) { distancepr = distance;} 
      
      var alpha = math.acos( (distancepr / distance) );
      var beta = ( dragheadto - gliderhead ) * 0.01745;
      var gamma = gliderpitch * 0.01745;
      var delta = gliderroll * 0.01745;
      
      var sina = math.sin(alpha);
      var cosa = math.cos(alpha);
      var sinb = math.sin(beta);
      var cosb = math.cos(beta);
      var sing = math.sin(gamma);
      var cosg = math.cos(gamma);
      var sind = math.sin(delta);
      var cosd = math.cos(delta);
      
      # global forces: alpha beta
      var fglobalx = forcetow * cosa * cosb;
      var fglobaly = forcetow * cosa * sinb;
      var fglobalz = forcetow * sina;
      if ( dragger.alt() > glider.alt()) {
        fglobalz = -fglobalz;
      }
      
      # local forces by pitch: gamma
      var flpitchx = fglobalx * cosg - fglobalz * sing;
      var flpitchy = fglobaly;
      var flpitchz = fglobalx * sing + fglobalz * cosg;
      
      # local forces by roll: delta
      var flrollx  = flpitchx;
      var flrolly  = flpitchy * cosd + flpitchz * sind;
      var flrollz  = flpitchy * sind + flpitchz * cosd;
      
      # asigning to LOCAL coord of plane
      var forcex = flrollx;
      var forcey = flrolly;
      var forcez = flrollz;
      
      # do all the stuff
      setprop("/fdm/jsbsim/external_reactions/dragx/magnitude",  forcex);
      setprop("/fdm/jsbsim/external_reactions/dragy/magnitude",  forcey);
      setprop("/fdm/jsbsim/external_reactions/dragz/magnitude",  forcez);
    }
    else {
      releaseDragger();
    }
  }
  
  settimer(runDragger, timeincrement);
  
} # End Function runDragger

var dragging = setlistener("/sim/glider/dragger/hooked", runDragger);
 
// Constants
var ROSBRIDGEPORT = 9090;
var MJPEGSERVERPORT = 8080;

// Globals
var segbots = {};
var segbot = null;
var servo1Cmd = null;
var servo2Cmd = null;
var servo1Pos = 0.0; // {-2,2} but locked to {-1,1}
var servo2Pos = 0.0;
var moveBaseAction = null;
var goToLocationClient = null;
var rotateClient = null;

// Map conversions
var map_res = 0.05;
var map_origin_x = -60.45;
var map_origin_y = -23.4;
var map_width = 1857;
var map_height = 573;
var map_marker_offset = 15; //half of image width

var locations = [{
  name: "Matteo's Office",
  value: "l3_418"
},{
  name: "Shiqi's Office",
  value: "l3_420"
},{
  name: "Peter's Office",
  value: "l3_415"
},{
  name: "Lab",
  value: "l3_414b"
}];



// objects
var Segbot = {
  name : "noname",
  ipaddr : "0.0.0.0",
  rosbridgeport : 9090,
  mjpegserverport : 0,
  ros : null,

  connect : function() {
    log("Trying to connect to " + this.ipaddr + " : " + this.rosbridgeport);
    this.ros = new ROSLIB.Ros({
      url : 'ws://' + this.ipaddr + ':' + this.rosbridgeport
    });
    this.ros.on('connection', function() { log('Connection made!'); });
    this.ros.on('error', function(err) {
      log("Connection failed!");
      log(err);
      error("Connection to segbot failed!");
    });
  }
}

// Functions
function error(errorMessage, errorTitle = "Oops! This is embarassing") {
  $("#errorTitle").text(errorTitle);
  $("#errorBody").text(errorMessage);
  $(".error-modal").modal();
  log("error: "+errorMessage);
}

function createSegbots() {
  segbots["localhost"] = createSegbot("localhost", "127.0.0.1", ROSBRIDGEPORT, MJPEGSERVERPORT);

  var server = "http://nixons-head.csres.utexas.edu:7979/hostsjson";
  if (server == "") {
    error("Will not be able to dynamically load robot's IP addresses","Error: No DNS server set");
    return;
  }
  log("Pinging dns server");
  $.getJSON(server, function(data) {
    $.each(data, function(key, val) {
      segbots[key] = createSegbot(key, val, ROSBRIDGEPORT, MJPEGSERVERPORT);
    });
  }).error(function(err) { error("Failed to ping DNS server"); });
}

function createSegbot(name, ipaddr, rosbridgeport, mjpegserverport) {
  var bot = Object.create(Segbot);
  bot.name = name;
  bot.ipaddr = ipaddr;
  bot.rosbridgeport = rosbridgeport;
  bot.mjpegserverport = mjpegserverport;
  log("Created segbot: " + name + "(" + ipaddr + ":" + rosbridgeport + ")");
  return bot;
}

function SegBotConnection(ipaddr, rosbridgeport, mjpegserverport) {
  this.ipaddr = ipaddr;
  this.rosbridgeport = rosbridgeport;
  this.mjpegserverport = mjpegserverport;
  this.ros = new ROSLIB.Ros({
    url : 'ws://' + ipaddr + ':' + rosbridgeport
  });
  log("Created new SegBotConnection with " + ipaddr +  ":" + rosbridgeport);
}

function populateLocations() {
  $(locations).each(function(i, val) {
    $('#locationSelect').append($('<option>', {
      value: val.value,
      text: val.name
    }));
  });
}


function subscribeListener(ros) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  });
  log("Added ping listener");

  listener.subscribe(function(message) {
    log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });
}

function subscribePoseListener(ros) {
  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : '/amcl_pose',
    messageType : 'geometry_msgs/PoseWithCovarianceStamped',
  });
  log("Added pose listener!");
  poseListener.subscribe(function(pose) {
    x = pose.pose.pose.position.x;
    y = pose.pose.pose.position.y;
    updatePosition(x,y);
  });
}

function updatePosition(x, y){
  log("robot.x = " + x + " robot.y = " + y);
  xp = 100 * (x / map_res + map_origin_x + map_marker_offset) / map_width;
  yp = 100 * (y / map_res + map_origin_y + map_marker_offset) / map_height;
  yp = 100 - yp;
  yp = yp - 10;
  log("robot.xp = " + xp + " robot.yp = " + yp);
  //yp = yp - 20;
  //yp = yp - 15; // offset for height of image and for css position
  $(".pos-marker").css("left", xp + "%");
  $(".pos-marker").css("top", yp + "%");
}

function publishTopic(ros) {
  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear : {
      x : 0.1,
      y : 0.2,
      z : 0.3
    },
    angular : {
      x : -0.1,
      y : -0.2,
      z : -0.3
    }
  });
  cmdVel.publish(twist);
}

function rotateLeft() {
  log("Rotate left");
  requestRotate(-5);
}

function rotateRight() {
  log("Rotate right");
  requestRotate(5);
}

function turnLeft() {
  servo1Pos += 0.2;
  servo1Pos = servo1Pos > 1.0 ? 1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log("Servo1: " + servo1Pos);
}

function turnRight() {
  servo1Pos -= 0.2;
  servo1Pos = servo1Pos < -1.0 ? -1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log("Servo1: " + servo1Pos);
}

function turnUp() {
  servo2Pos -= 0.2;
  servo2Pos = servo2Pos < 0.4 ? 0.4 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log("Servo2: " + servo2Pos);
}

function turnDown() {
  servo2Pos += 0.2;
  servo2Pos = servo2Pos > 2.0 ? 2.0 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log("Servo2: " + servo2Pos);
}
function turnCenter() {
  servo1Pos = 0.0;
  servo2Pos = 0.0;
  servo1Cmd.publish(new ROSLIB.Message({data: 0.0}));
  servo2Cmd.publish(new ROSLIB.Message({data: 1.0}));
  log("Servo1: " + servo1Pos);
  log("Servo2: " + servo2Pos);
}

function showMap() {
  log("Showing map");
  $(".map-modal").modal();
}

function sendGoal(pose) {
  var goal = new ROSLIB.Goal({
    actionClient : moveBaseAction,
    goalMessage : {
      target_pose : {
        header : {
          frame_id : '/base_footprint'
        },
        pose : pose
      }
    }
  });

  goal.on('result', function(result) {
    log("move_base result: " + result);
  });
  goal.send();
}

function requestLocation(locationStr) {
  log('requesting goToLocation: ' + locationStr);
  var request = new ROSLIB.ServiceRequest({ location: locationStr});
  goToLocationClient.callService(request, function(result) {
    log('Result for requestLocation service call on '
      + goToLocationClient.name + ': ' + result.result);
    if (result.result == 1) { //success
      alert("success");
    } else if (result.result == -1) { // terminated
      alert("terminated");
    } else if (result.result == -2) { // preempted
      alert("preempted");
    } else if (result.result == -3) { // aborted
      alert("aborted");
    } else {
    }
  });
}

function requestRotate(rotateDelta) {
  log('requesting rotate: ' + rotateDelta);
  var request = new ROSLIB.ServiceRequest({ rotateDelta: rotateDelta});
  rotateClient.callService(request, function(result) {
    log('Result for requestRotate service call on '
      + rotateClient.name + ': ' + result.result);
  });
}


// Handlers
$(document).ready(function() {
  log("Loaded.");

  log("Creating segbots");
  createSegbots();

  populateLocations();
});

$(".robot").click(function() {
  var botname = $(this).attr("robot");
  segbot = segbots[botname];

  log("Selected: " + botname); 
  segbot.connect();

  log("Subscribing listeners");
  subscribeListener(segbot.ros);
  subscribePoseListener(segbot.ros);

  // hide the intro stuff
  $(".intro").fadeOut();
  $(".robots").fadeOut();

  // set up title
  $(".controllingText").text("Controlling " + botname);

  // set up video streaming
  var videoTopic = "";
  if (botname == "calculon") {
    videoTopic = "/camera/image_raw";
    videoTopic += "?invert";
    log("Using /camera/image_raw for video source");
  } else {
    videoTopic = "/nav_kinect/rgb/image_raw";
    log("Using /nav_kinect/rgb/image_raw for video source");
  }
  var videoSource = "http://" + segbot.ipaddr + ":" + segbot.mjpegserverport
                      + "/stream?topic=" + videoTopic;
  log("Loading video from: " + videoSource);
  $(".controllingIframe").append("<img width=\"100%\" height=\"800\" src=\"" + videoSource + "\">");

  // set up topic for controlling servo
  servo1Cmd = new ROSLIB.Topic({
    ros : segbot.ros,
    name : '/servo0_cmd',
    messageType : 'std_msgs/Float32'
  });
  servo2Cmd = new ROSLIB.Topic({
    ros : segbot.ros,
    name : '/servo1_cmd',
    messageType : 'std_msgs/Float32'
  });

  // set up actionlib for teleop moving
  moveBaseAction = ROSLIB.ActionClient({
    ros : segbot.ros,
    serverName : '/move_base',
    actionName : 'move_base_msgs/MoveBaseAction'
  });

  // set up service client for sending location commands
  goToLocationClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/go_to_location',
    serviceType : 'bwi_virtour/GoToLocation'
  });

  // set up service client for sending location commands
  rotateClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/rotate',
    serviceType : 'bwi_virtour/Rotate'
  });

//  var pose = ROSLIB.Pose({position : { x : 1, y : 1 }});
//  sendGoal(pose);

  // reset the servo
  turnCenter();

  // show the robot stuff
  $(".control").delay(800).fadeIn();
});

// add callback handlers for buttons
$(".turnLeft").click(function() {turnLeft();});
$(".turnRight").click(function() {turnRight();});
$(".turnUp").click(function() {turnUp();});
$(".turnDown").click(function() {turnDown();});
$(".turnCenter").click(function() {turnCenter();});
$(".labimage").click(function() {showMap();});
$(".rotateRight").click(function() {rotateRight();});
$(".rotateLeft").click(function() {rotateLeft();});

// add callback handlers for navigate form
$(".navigateBtn").click(function() {
  var location = $("#locationSelect").val();
  log("Requesting navigation to " + location);
  requestLocation(location);
  $(".map-modal").modal("hide");
});

$(".reloadBtn").click(function() {
  log("reloading page");
  location.reload();
});

// map arrow keys to movement
$(document).keypress(function(e) {
  if (e.keyCode === 37) {
    turnLeft();
  } else if (e.keyCode === 39) {
    turnRight();
  } else if (e.keyCode === 38) {
    turnUp();
  } else if (e.keyCode === 40) {
    turnDown();
  }
});

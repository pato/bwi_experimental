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

// objects
var Segbot = {
  name : "noname",
  ipaddr : "0.0.0.0",
  rosbridgeport : 9090,
  mjpegserverport : 0,
  ros : null,

  connect : function() {
    log("Created connection: " + this.ipaddr + " : " + this.rosbridgeport);
    this.ros = new ROSLIB.Ros({
      url : 'ws://' + this.ipaddr + ':' + this.rosbridgeport
    });
  }
}

// Functions
function createSegbots() {
  //segbots["localhost"] = createSegbot("localhost", "127.0.0.1", ROSBRIDGEPORT, MJPEGSERVERPORT);

  var server = "http://nixons-head.csres.utexas.edu:7979/hostsjson";
  if (server == "") {
    log("Warning: No DNS server set, will not be able to dynamically load robot's IP addresses");
    return;
  }
  log("Pinging dns server");
  $.getJSON(server, function(data) {
    $.each(data, function(key, val) {
      segbots[key] = createSegbot(key, val, ROSBRIDGEPORT, MJPEGSERVERPORT);
    });
  });
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


function subscribeListener(ros) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  });
  log("Added listener");

  listener.subscribe(function(message) {
    log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });
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

function turnLeft() {
  servo1Pos += 0.2;
  servo1Pos = servo1Pos > 1.0 ? 1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log(servo1Pos);
}

function turnRight() {
  servo1Pos -= 0.2;
  servo1Pos = servo1Pos < -1.0 ? -1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log(servo1Pos);
}

function turnUp() {
  servo2Pos -= 0.2;
  servo2Pos = servo2Pos < 0.2 ? 0.2 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log(servo2Pos);
}

function turnDown() {
  servo2Pos += 0.2;
  servo2Pos = servo2Pos > 2.0 ? 2.0 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log(servo2Pos);
}
function turnCenter() {
  servo1Pos = 0.0;
  servo2Pos = 0.0;
  servo1Cmd.publish(new ROSLIB.Message({data: 0.0}));
  servo2Cmd.publish(new ROSLIB.Message({data: 1.0}));
}

// Handlers
$(document).ready(function() {
  log("Loaded.");

  log("Creating segbots");
  createSegbots();
});

$(".robot").click(function() {
  var botname = $(this).attr("robot");
  segbot = segbots[botname];//segbots['localhost'];//segbots[botname];

  log(segbot);

  log("Connecting to: " + botname); 
  segbot.connect();

  log("Subscribing listener");
  subscribeListener(segbot.ros);

  // hide the intro stuff
  $(".intro").fadeOut();
  $(".robots").fadeOut();

  // set up title
  $(".controllingText").text("Controlling " + botname);

  // set up video streaming
  //var videoTopic = "/nav_kinect/rgb/image_raw";
  var videoTopic = "/camera/image_raw";
  var videoSource = "http://" + segbot.ipaddr + ":" + segbot.mjpegserverport
                      + "/stream?topic=" + videoTopic + "?invert";
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

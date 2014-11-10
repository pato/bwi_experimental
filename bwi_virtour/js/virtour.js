// Constants
var ROSBRIDGEPORT = 9090;
var MJPEGSERVERPORT = 8080;

// Globals
var segbots = {};
var segbot = null;
var servoCmd = null;
var servoPos = 0.0; // {-2,2} but locked to {-1,1}

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
                      + "/stream?topic=" + videoTopic;
  log("Loading video from: " + videoSource);
  $(".controllingIframe").append("<img width=\"100%\" height=\"800\" src=\"" + videoSource + "\">");

  // set up topic for controlling servo
  servoCmd = new ROSLIB.Topic({
    ros : segbot.ros,
    name : '/servo0_cmd',
    messageType : 'std_msgs/Float32'
  });
  
  // reset the servo
  servoCmd.publish(new ROSLIB.Message({data: 0.0}));

  // show the robot stuff
  $(".control").delay(800).fadeIn();
});
$(".turnLeft").click(function() {
  servoPos += 0.2;
  servoPos = servoPos > 1.0 ? 1.0 : servoPos;
  servoCmd.publish(new ROSLIB.Message({data: servoPos}));
});
$(".turnRight").click(function() {
  servoPos -= 0.2;
  servoPos = servoPos < -1.0 ? -1.0 : servoPos;
  servoCmd.publish(new ROSLIB.Message({data: servoPos}));
});
$(document).keypress(function(e) {
  if (e.keyCode === 37) {
    servoPos += 0.2;
    servoPos = servoPos > 1.0 ? 1.0 : servoPos;
    servoCmd.publish(new ROSLIB.Message({data: servoPos}));
  } else if (e.keyCode === 39) {
    servoPos -= 0.2;
    servoPos = servoPos < -1.0 ? -1.0 : servoPos;
    servoCmd.publish(new ROSLIB.Message({data: servoPos}));
  }
});

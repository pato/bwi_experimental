// constants
var ROSBRIDGEPORT = 9090;
var MJPEGSERVERPORT = 8888;

// Globals
var segbots = {};

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

// functions
function createSegbots() {
  segbots["localhost"] = createSegbot("localhost", "127.0.0.1", ROSBRIDGEPORT, MJPEGSERVERPORT);

  var server = "http://nixons-head.csres.utexas.edu:7979/hostsjson";
  if (server == "") {
    log("Warning: No DNS server set, will not be able to dynamically load robot's IP addresses");
    return;
  }
  log("Pinging dns server");
  $.getJSON(server, function(data) {
    $.each(data, function(key, val) {
      //connectionConfig[key] = createConnectionConfig(val, 9090, 8080)
      //$('#connectionSelect').append('<option value="' + key + '">' + key + '</option>');
      //alert(key + " : " + val);
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

function getHostAddresses() {
  var server = "http://nixons-head.csres.utexas.edu:7979/hostsjson";
  if (server == "") {
    log("Warning: No DNS server set, will not be able to dynamically load robot's IP addresses");
    return;
  }
  $.getJSON(server, function(data) {
    $.each(data, function(key, val) {
      //connectionConfig[key] = createConnectionConfig(val, 9090, 8080)
      //$('#connectionSelect').append('<option value="' + key + '">' + key + '</option>');
      alert(key + " : " + val);
    });
  });
}

$(document).ready(function() {
  log("Loaded.");

  log("Creating segbots");
  createSegbots();
  /*
  var seg = new SegBotConnection("localhost", 9090);

  log("Trying to get ip addresses");
  getHostAddresses();

  subscribeListener(seg.ros);
  log("Subscribed");
  
  publishTopic(seg.ros);
  log("Published command");
  */
});

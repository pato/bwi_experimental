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
  var server = "";
  if (server == "") {
    console.log("Warning: No DNS server set, will not be able to dynamically load robot's IP addresses");
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
  log("Loaded. Starting first connection...");
  var seg = new SegBotConnection("localhost", 9090);

  log("Trying to get ip addresses");
  getHostAddresses();

  subscribeListener(seg.ros);
  log("Subscribed");
  
  publishTopic(seg.ros);
  log("Published command");
});

function SegBotConnection(ipaddr, rosbridgeport, mjpegserverport) {
  this.ipaddr = ipaddr;
  this.rosbridgeport = rosbridgeport;
  this.mjpegserverport = mjpegserverport;
  this.ros = new ROSLIB.Ros({
    url : 'ws://' + ipaddr + ':' + rosbridgeport
  });
  log("Created new SegBotConnection with " + ipaddr +  ":" + rosbridgeport);
}

function log(str) {
  console.log(str);
  $("#consoleout").append(str + "\n");
}

$(document).ready(function() {
  var seg = new SegBotConnection("localhost", 9090);

  var listener = new ROSLIB.Topic({
    ros : seg.ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  });

  listener.subscribe(function(message) {
    log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });

});

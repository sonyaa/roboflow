var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
  });


var navPub = new ROSLIB.Topic({
	ros : ros,
	name : '/navigation_command',
	messageType : 'rws_pr2_navigation/NavigationCommand'
});


var expListener = new ROSLIB.Topic({
	ros : ros,
	name : '/nav_system_state',
	messageType : 'rws_pr2_navigation/NavSystemState'
});

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_nav_system_state',
	serviceType : 'pr2_pbd_interaction/GetNavSystemState'
});

function init() {
    ros.on("error", function() {
        alert("Error connecting to the ROS server. App will not work.")
    });

    storedLocations = getStoredLocations();
    storedHeadPoses = getStoredHeadPoses();
    storedManipulationActions = getStoredManipulationActions();

//
//	expListener.subscribe(function(state) {
//		drawState(state);
//	});
//
//	estimatedPoseListener.subscribe(function(poseArray) {
//		processPoseArray(poseArray);
//	});
//
//	expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
//		drawState(result.state);
//	});


	//
  }
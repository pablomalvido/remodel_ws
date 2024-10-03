vue3 = new Vue({
  el:'#vue-app3', //This vue instance is applied to the elements inside the container tag with id vue-app2

  //Vue instance (vue2) variables below
  data: {
    ros: null,
    connectedTxt: "ROS connection not stablished",
    alarm_title: "ALARMS",
    active_alarms: [],
    inactive_alarms: [],
    problems: {"robot": {"detected": false, "active": false}, "Estop": {"detected": false, "active": false}, "curtain": {"detected": false, "active": false}, "door": {"detected": false, "active": false}, "UI": {"detected": false, "active": false}, "vision": {"detected": false, "active": false}},
  },

  //Vue instance (vue3) methods below
  methods:{
    connect: function(){

	if(this.connectedTxt=="ROS connection not stablished" || this.connectedTxt=="ERROR"){
                ros = new ROSLIB.Ros({
                    url : 'ws://192.168.43.168:9090' //Port = 9090
                });

                //All the code inside ros.on is executed when the connection with ROS is stablished
                ros.on('connection', function() {
                    vue3.connectedTxt="ROS connection stablished";
                    console.log('Connected to websocket server.');

		    //Subscribe to several topics
                    //Subscribe to the topic that publishes the reset
                    listener_reset = new ROSLIB.Topic({
                      ros : ros,
                      name : '/RSM/reset',  // /RSM/ROS_reset for tests
                      messageType : 'std_msgs/Bool' //Change this for your topic type
                    });

		    listener_reset.subscribe(function(message) {
                      console.log('Received message on ' + listener_reset.name + ': ' + message.data);
		      /*for(key in vue3.problems){
			vue3.problems[key]["detected"] = false;
			vue3.problems[key]["active"] = false;
    		      }*/
		      if(message.data){
			vue3.problems = {"robot": {"detected": false, "active": false}, "Estop": {"detected": false, "active": false}, "curtain": {"detected": false, "active": false}, "door": {"detected": false, "active": false}, "UI": {"detected": false, "active": false}, "vision": {"detected": false, "active": false}};
			vue3.active_alarms = [];
			vue3.inactive_alarms = [];
			console.log("Alarms reseted");
		      }
                    });

		    //Subscribe to the topic that publishes the alarms
                    listener_alarms = new ROSLIB.Topic({
                      ros : ros,
                      name : '/RSM/problems',  //Change this for your topic name
                      messageType : 'remodel_safety_manager/RSM_problems' //Change this for your topic type
                    });

		    listener_alarms.subscribe(function(message) {
                      console.log('Received message on ' + listener_alarms.name + ': ' + message.Estop);

		      if(message.Estop){
			vue3.problems["Estop"]["detected"] = true
			vue3.problems["Estop"]["active"] = true
		      } else {
			vue3.problems["Estop"]["active"] = false
		      }
		      if(message.door){
			vue3.problems["door"]["detected"] = true
			vue3.problems["door"]["active"] = true
		      } else {
			vue3.problems["door"]["active"] = false
		      }
		      if(message.curtain){
			vue3.problems["curtain"]["detected"] = true
			vue3.problems["curtain"]["active"] = true
		      } else {
			vue3.problems["curtain"]["active"] = false
		      }
		      if(message.robot){
			vue3.problems["robot"]["detected"] = true
			vue3.problems["robot"]["active"] = true
		      } else {
			vue3.problems["robot"]["active"] = false
		      }
		      if(message.UI){
			vue3.problems["UI"]["detected"] = true
			vue3.problems["UI"]["active"] = true
		      } else {
			vue3.problems["UI"]["active"] = false
		      }
		      if(message.vision){
			vue3.problems["vision"]["detected"] = true
			vue3.problems["vision"]["active"] = true
		      } else {
			vue3.problems["vision"]["active"] = false
		      }

		      active_alarms_temp = [];
		      inactive_alarms_temp = [];
		      if(vue3.problems["Estop"]["detected"]){
			if(vue3.problems["Estop"]["active"]){
			  active_alarms_temp.push("Pressed emergency stop button")
			} else {
			  inactive_alarms_temp.push("Pressed emergency stop button (inactive)")
			}
		      }
		      if(vue3.problems["door"]["detected"]){
			if(vue3.problems["door"]["active"]){
			  active_alarms_temp.push("The door of the cell was opened")
			} else {
			  inactive_alarms_temp.push("The door of the cell was opened (inactive)")
			}
		      }
		      if(vue3.problems["curtain"]["detected"]){
			if(vue3.problems["curtain"]["active"]){
			  active_alarms_temp.push("Light curtain detection")
			} else {
			  inactive_alarms_temp.push("Light curtain detection (inactive)")
			}
		      }
		      /*if(vue3.problems["robot"]["detected"]){
			if(vue3.problems["robot"]["active"]){
			  active_alarms_temp.push("Robot's problem")
			} else {
			  inactive_alarms_temp.push("Robot's problem (inactive)")
			}
		      }*/
		      if(vue3.problems["UI"]["detected"]){
			if(vue3.problems["UI"]["active"]){
			  active_alarms_temp.push("User Interface emergency stop button")
			} else {
			  inactive_alarms_temp.push("User Interface emergency stop button (inactive)")
			}
		      }
		      if(vue3.problems["vision"]["detected"]){
			if(vue3.problems["vision"]["active"]){
			  active_alarms_temp.push("Vision system alarm")
			} else {
			  inactive_alarms_temp.push("Vision system alarm (inactive)")
			}
		      }
		      vue3.active_alarms = active_alarms_temp;
		      vue3.inactive_alarms = inactive_alarms_temp;

                      console.log("Alarms updated");
                    });

		    //Subscribe to the topic that publishes the safety_ok
                    listener_safety = new ROSLIB.Topic({
                      ros : ros,
                      name : '/RSM/safety_ok',  //Change this for your topic name
                      messageType : 'std_msgs/Bool' //Change this for your topic type
                    });

		    listener_safety.subscribe(function(message) {
                      console.log('Received message on ' + listener_safety.name + ': ' + message.data);
		      if(message.data){
			vue3.alarm_title = "NO ALARMS"
		      } else {
			vue3.alarm_title = "ALARMS"
		      }			
                      console.log("Safety updated");
                    });

		    //Publishes a signal so the RSM publishes the alarms again
		    var loadPublisher = new ROSLIB.Topic({
        		ros : ros,
        		name : '/RSM/load_page',
        		messageType : 'std_msgs/Bool'
      		    });

      		    var loadTopic = new ROSLIB.Message({
        		data: true
      		    });

		    loadPublisher.publish(loadTopic);
		});

		ros.on('error', function(error) {
                    console.log('Error connecting to websocket server: ', error);
                    vue3.connectedTxt="ROS connection error";
                });
                
                ros.on('close', function() {
                    console.log('Connection to websocket server closed.');
                    vue3.connectedTxt="ROS connection closed";
                });
            }
            else if(this.connectedTxt=="CONNECTED"){
                ros.close();
            }
    },

    reset_alarms: function(){
      //vue3.active_alarms = [];
      //vue3.inactive_alarms = [];
      console.log("Reset function");
      var resetPublisher = new ROSLIB.Topic({
        ros : ros,
        name : '/RSM/ROS_reset',
        messageType : 'std_msgs/Bool'
      });

      var resetTopic = new ROSLIB.Message({
        data: true
      });

      resetPublisher.publish(resetTopic);
    }
  },

  computed:{

  }
});

window.addEventListener('load', function () {
  vue3.connect()
});

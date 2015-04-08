senseGui.controller("configController",['$scope',function($scope){
  $scope.maxAddress = 7;
  $scope.alerts = [];
  $scope.test = false;
  $scope.portOptions = [
      {
        name:"A1",
        type:"A"
      },
       {
        name:"A2",
        type:"A"
      },
       {
        name:"A3",
        type:"A"
      },
       {
        name:"A4",
        type:"A"
      },
       {
        name:"D1",
        type:"D"
      },
       {
        name:"D2",
        type:"D"
      }
    ]
  $scope.intervalOptions = ["second", "minute", "hour", "day", "week"];
  $scope.types = sensorTypes;
	
  $scope.changePort = function(){
    angular.forEach($scope.portOptions,function(v,i){
      if($scope.sensor.type.connCode=="A" && v.name == "A1")
        $scope.sensor.port = v;
      if($scope.sensor.type.connCode == "D" && v.name == "D1")
        $scope.sensor.port = v;
    })
  }

  $scope.sensor = {
          id:null,
        name:"",
        type:$scope.types[0],
		    port:$scope.portOptions[0],
		 address:0,
		 samples:0,
		interval:$scope.intervalOptions[0]
  }
  
	

  $scope.config = {
    id:null,
    name:"",
    overwrite:false,
    sensors:[]
  };

  /*
	
	$scope.config.sensors.push({id:0,name:"thing1",type:$scope.types[0],port:$scope.portOptions[4],address:0,samples:1,interval:$scope.intervalOptions[0]});
	$scope.config.sensors.push({id:3,name:"thing49",type:$scope.types[0],port:$scope.portOptions[4],address:0,samples:1,interval:$scope.intervalOptions[0]});
	$scope.config.sensors.push({id:2,name:"thing3",type:$scope.types[0],port:$scope.portOptions[4],address:0,samples:1,interval:$scope.intervalOptions[0]});
	*/
	
  $scope.addSensor = function(){
	  // VALIDATION
    $scope.alerts = [];
	// No name
    if($scope.sensor.name == "")
      $scope.alerts.push("Please name the sensor.");
  
	// Invalid sampling rate
	if($scope.sensor.samples <= 0)
      $scope.alerts.push("Please enter a sampling rate.");
  
	// Address
    if($scope.sensor.type.connCode == "D")
    {
		// valid number
      if(!/^\d+$/.test($scope.sensor.address))
        $scope.alerts.push("Address must be a number.")
		// In window
      else if(parseInt($scope.sensor.address) < 0 || parseInt($scope.sensor.address) >= $scope.maxAddress)
        $scope.alerts.push("Address must be between 0 and " + $scope.maxAddress + ".");
    }
    else{
      $scope.sensor.address = null;
    }

    angular.forEach($scope.config.sensors,function(v,i){
		// No duplicate name
		if($scope.sensor.name == v.name){
			$scope.alerts.push("There is already a sensor named \"" + v.name + "\".");
		}
        if($scope.sensor.id == null || $scope.sensor.id != v.id)
        {
			// No duplicate address / port combo
			if($scope.sensor.type.connCode == "A")
			{
				if(v.port.name == $scope.sensor.port.name)
				$scope.alerts.push("Analog port " + v.port.name + " occupied by sensor \"" + v.name + "\".");
			}
			else
			{
				if(v.port.name == $scope.sensor.port.name && v.address == $scope.sensor.address)
				$scope.alerts.push("Port/address " + v.port.name + "." + v.address + " occupied by sensor \"" + v.name + "\".");
			}
        }
    })

	  // Passed validation, save sensor
    if($scope.alerts.length == 0){
      if($scope.sensor.id == null){
        $scope.sensor.id = $scope.config.sensors.length+1;
        $scope.config.sensors.push(angular.copy($scope.sensor));
      }
      else{
        var idx = null;
        angular.forEach($scope.config.sensors,function(v,i){
          if(v.id == $scope.sensor.id)
            idx = i;
        })
        $scope.config.sensors[idx] = angular.copy($scope.sensor);
      }

      $scope.resetSensor();
    }
  }
  
  $scope.deleteSensor = function(index){
	$scope.config.sensors.splice(index,1);
  }
  
  $scope.editSensor = function(sensor){
	  var tmpName = sensor.name
	  sensor.name = "*" + sensor.name;
    $scope.sensor = angular.copy(sensor);
	$scope.sensor.name = tmpName;
    angular.forEach($scope.types,function(v,i){
      if(v.id == sensor.type.id)
        $scope.sensor.type = v;
    })
    angular.forEach($scope.portOptions,function(v,i){
      if(v.name == sensor.port.name)
        $scope.sensor.port = v;
    })


  }

  $scope.resetSensor = function(){
      $scope.sensor.id = null;
      $scope.sensor.name = "";
      //$scope.sensor.port = ;
      $scope.sensor.address = null;
  }

  $scope.saveConfig = function(){
	  console.log($scope.config);
		var config = {type: 'saveFile',suggestedName:'config', accepts:[{extensions:["cfg"]}]};
		chrome.fileSystem.chooseEntry(config, function(writableEntry) {	
			if (!writableEntry) {
				console.log("No entry");
				return;
			}
			writableEntry.createWriter(function(writer) {
				writer.onerror = function(e){console.error(e);};
				writer.onwriteend = function(e) {console.log('Write complete');};
				waitForIO(writer, function(){
					writer.seek(0);
					writer.write(new Blob([angular.toJson($scope.config)], {type:"text/plain"}));
				});
			});
		});
	}
	
  $scope.loadConfig = function(){
	  
		var config = {type: 'openFile'};
		chrome.fileSystem.chooseEntry(config, function(readOnlyEntry) {

		readOnlyEntry.file(function(file) {
		  var reader = new FileReader();

		  reader.onloadend = function(e) {
			$scope.config = angular.fromJson(e.target.result);
			console.log($scope.config);
		  };

		  var arg = reader.readAsText(file);
		});
  })};
	
	function waitForIO(writer, callback) {
	  // set a watchdog to avoid eventual locking:
	  var start = Date.now();
	  // wait for a few seconds
	  var reentrant = function() {
		if (writer.readyState===writer.WRITING && Date.now()-start<4000) {
		  setTimeout(reentrant, 100);
		  return;
		}
		if (writer.readyState===writer.WRITING) {
		  console.error("Write operation taking too long, aborting!"+
			" (current writer readyState is "+writer.readyState+")");
		  writer.abort();
		} 
		else {
		  callback();
		}
	  };
	  setTimeout(reentrant, 100);
	}
	
	$scope.launchSerial = function(){
		console.log("launch serial");
		chrome.app.window.create(
			'connect.html',
			{
			  id: 'connectWindow',
			  bounds: {width: 500, height: 500}
			},
			function(createWindow){
				console.log("created window");
			}
		);
	}
	

}]);





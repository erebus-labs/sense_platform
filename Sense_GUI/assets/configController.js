senseGui.controller("configController",['$scope',function($scope){
  $scope.editing = false;
  $scope.programming = false;
  $scope.maxAddress = 7;
  $scope.alerts = [];
  $scope.test = false;
  $scope.portOptions = ["A1","A2","A3","A4"];
  $scope.intervalOptions = ["second", "minute", "hour", "day", "week"];
  $scope.types = sensorTypes;

  $scope.sensor = {
			  id:null,
			name:"",
			type:$scope.types[0],
		    port:$scope.portOptions[0],
		 address:0,
		 samples:1,
		interval:$scope.intervalOptions[0]
  }
  
  $scope.cancel = function(){
	  $scope.editing = false;
	  $scope.programming = false;
  }
  
  $scope.cancelProgramming = function(){
	  $scope.programming = false;
	  serial.Disconnect();
  }
  
  	$scope.newSensor = function(){
		$scope.editing = true;
		$scope.resetSensor();
	}
	

  $scope.config = {
    id:null,
    name:"",
    overwrite:false,
    sensors:[]
  };

  /*
	
	$scope.config.sensors.push({id:0,name:"thing1",type:$scope.types[0],port:$scope.portOptions[0],address:0,samples:1,interval:$scope.intervalOptions[0]});
	$scope.config.sensors.push({id:3,name:"thing2",type:$scope.types[0],port:$scope.portOptions[1],address:0,samples:2,interval:$scope.intervalOptions[1]});
	$scope.config.sensors.push({id:2,name:"thing3",type:$scope.types[1],port:$scope.portOptions[3],address:0,samples:3,interval:$scope.intervalOptions[2]});
	
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
      $scope.sensor.port = null;
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
				if(v.port == $scope.sensor.port)
				$scope.alerts.push("Analog port " + v.port + " occupied by sensor \"" + v.name + "\".");
			}
			else
			{
				if(v.address == $scope.sensor.address)
				$scope.alerts.push("Address " + v.address + " occupied by sensor \"" + v.name + "\".");
			}
        }
    })

	  // Passed validation, save sensor
    if($scope.alerts.length == 0){
		$scope.editing = false;
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
	  $scope.editing = true;
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
			$scope.$apply();
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
		$scope.programming = true;
	}
	
	$scope.pushConfig = function(){
		log("Pushing Config...");
	}

}]);





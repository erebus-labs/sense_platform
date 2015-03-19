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
  $scope.digAddressCount = 8;
  $scope.intervalOptions = ["second", "minute", "hour", "day", "week"];
  $scope.types = [{
    name:"Volatile Organic Compound",
    connCode:'ADC'}
    ,
    {name:"Accelerometer",
    connCode:'I2C'
    }];

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
    sensors:[],
  };

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
  
  $scope.editSensor = function(sensor){
	  var tmpName = sensor.name
	  sensor.name = "*" + sensor.name;
    $scope.sensor = angular.copy(sensor);
	$scope.sensor.name = tmpName;
    angular.forEach($scope.types,function(v,i){
      if(v.name == sensor.type.name)
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
	  var config = {type: 'saveFile', suggestedName: chosenEntry.name};
	  chrome.fileSystem.chooseEntry(config, function(writableEntry) {
		var blob = new Blob([textarea.value], {type: 'text/plain'});
		writeFileEntry(writableEntry, blob, function(e) {
		  output.textContent = 'Write complete :)';
		});
	  });
  }
  
  $scope.loadConfig = function(){
	  console.log("trying to load...");
	$.getJSON("manifest.json", function(json) {
		console.log(json); // this will show the info it in firebug console
	});
  }

}]);

















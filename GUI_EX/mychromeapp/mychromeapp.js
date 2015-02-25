var MY_HID_VENDOR_ID  = 0x0d62; // 4660 in hexadecimal!
var MY_HID_PRODUCT_ID = 0x001c;
var DEVICE_INFO = {"vendorId": MY_HID_VENDOR_ID, "productId": MY_HID_PRODUCT_ID };

var connectionId = null;

function arrayBufferToString(array) {
  return String.fromCharCode.apply(null, new Uint8Array(array));
}

var myDevicePoll = function() {
  var size = 64;

	chrome.hid.receive(connectionId, size, function(data) {
	  if (data != null) {
			// Convert Byte into Ascii to follow the format of our device
			myText.value = arrayBufferToString(data);
			console.log('Data: ' + myText.value);
	  }

    setTimeout(myDevicePoll, 0);
	});
}

function initializeHid(pollHid) {
	chrome.hid.getDevices(DEVICE_INFO, function(devices) {
		if (!devices || !devices.length) {
		  console.log('device not found');
		  return;
		}
		console.log('Found device: ' + devices[0].deviceId);
		myHidDevice = devices[0].deviceId;

		// Connect to the HID device
		chrome.hid.connect(myHidDevice, function(connection) {
			console.log('Connected to the HID device!');
		  connectionId = connection.connectionId;

			// Poll the USB HID Interrupt pipe
			pollHid();
		});
	});
}

initializeHid(myDevicePoll);

console.log("My App is running ...");

var myText = document.getElementById("mytext");
myText.value = "Youpi!";


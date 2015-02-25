chrome.app.runtime.onLaunched.addListener(function() {
  chrome.app.window.create('mychromeapp.html', {
    singleton: true,
    id: "My First Chrome Application"
  });
});


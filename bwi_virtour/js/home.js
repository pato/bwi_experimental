// set the log out
$(".info textarea").css("width", $(document).width() + "px");

// hide the debug
$(".info textarea").hide();

// toggle the debug if clicked
$(".showhide").click(function() {
  $(".info textarea").slideToggle();
  $(".showhide").text($(".showhide").text() == "Show debug" ? "Hide debug" : "Show debug");
});

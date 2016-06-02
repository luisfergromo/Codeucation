<?php
$botToken = "224238361:AAFViBooLN91Ob1FJQDQR-DypG4uPqdD8Y8";
$website = "https://api.telegram.org/bot".$botToken;

$update = file_get_contents($website."/getupdates");

print_r($update);

?>

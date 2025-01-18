<?php
    session_start();  // Start the session

    if (isset($_POST['id'])) {
        $_SESSION['id'] = $_POST['id'];  // Set the session variable 'id'
    }
?>

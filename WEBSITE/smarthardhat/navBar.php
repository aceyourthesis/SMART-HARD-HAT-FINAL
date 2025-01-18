 <div class="container ">
<!-- Navbar -->
  <nav class="navbar navbar-light  bg-success border-bottom rounded-bottom" >
    <div class="container-fluid d-flex justify-content-">
      <!-- Hamburger Button -->
      <button class="btn bg-light" type="button" data-bs-toggle="offcanvas" data-bs-target="#offcanvasNavbar" aria-controls="offcanvasNavbar" >
        <span class="navbar-toggler-icon" ></span>
      </button>
      <div class="d-flex justify-content-center flex-grow-1">
        <a class="navbar-brand text-white mx-3" href="#" style = "font-size: 2rem">
          <i class="fas fa-hard-hat me-2 " ></i> Smart Hard Hat Monitoring System
        </a>
      </div>
    </div>
  </nav>

    <!-- Offcanvas Sidebar -->
    <div class="offcanvas offcanvas-start" tabindex="-1" id="offcanvasNavbar" aria-labelledby="offcanvasNavbarLabel" style="width: 250px;">
      <div class="offcanvas-header">
        <h5 class="offcanvas-title" id="offcanvasNavbarLabel">Menu</h5>
        <button type="button" class="btn-close text-reset" data-bs-dismiss="offcanvas" aria-label="Close"></button>
      </div>
      <div class="offcanvas-body">
        <ul class="navbar-nav justify-content-start flex-grow-1 pe-3">
          <li class="nav-item"> <a class="nav-link active" aria-current="page" href="dashboard.php">Dashboard</a></li>
          <li class="nav-item"> <a class="nav-link" href="logs.php">Logs</a></li>
          <li class="nav-item"> <a class="nav-link" href="logout.php">Logout</a></li>
        </ul>
      </div>
    </div>

  </div>

<?php
    session_start();  // Start the session

    // Check if the user is logged in (i.e., session variable 'id' is set)
    if (!isset($_SESSION['id'])) {
        // Redirect to the login page if not logged in
        header('Location: login.php');
        exit();
    }
?>



<?php include 'head.php'; ?>

<div class="container d-flex justify-content-center align-items-center vh-100">
    <div class="card shadow-lg" style="max-width: 400px; width: 100%;">
        <div class="card-header text-center">
            <h4>Admin Login</h4>
        </div>
        <div class="card-body">
            <form id="adminLoginForm">
                <!-- Email Input -->
                <div class="mb-3">
                    <label for="email" class="form-label mb-0">Email address</label>
                    <input type="email" class="form-control" id="email" placeholder="Enter your email" required>
                </div>

                <!-- Password Input with Show Password Checkbox -->
                <div class="mb-3 d-flex flex-column">
                    <div class="d-flex justify-content-between align-items-center">
                        <label for="password" class="form-label mb-0">Password</label> 
                        <div>
                            <label for="showPassword" class="form-check-label">Show Password</label>
                            <input type="checkbox" id="showPassword" class="form-check-input" />
                        </div>
                    </div>
                    <input type="password" class="form-control" id="password" placeholder="Enter your password" required>
                </div>
                <!-- Submit Button -->
                <button type="submit" class="btn btn-primary w-100" id="loginBtn">Login</button>
            </form>
        </div>
        <div class="card-footer text-center">
            <small>&copy; 2025 CvSu - Main</small>
        </div>
    </div>
</div>

<!-- Show Password Script -->
<script type="module">
    // Import the necessary Firebase SDKs
    import { initializeApp } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-app.js";
    import { getAnalytics } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-analytics.js";
    import { getAuth, signInWithEmailAndPassword } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-auth.js";

    // Firebase configuration
    const firebaseConfig = {
        apiKey: "AIzaSyAy_bQFynVXe_RflYLYgsU0skc8ThOKDYE",
        authDomain: "smarthardhat-22267.firebaseapp.com",
        databaseURL: "https://smarthardhat-22267-default-rtdb.asia-southeast1.firebasedatabase.app",
        projectId: "smarthardhat-22267",
        storageBucket: "smarthardhat-22267.appspot.com",
        messagingSenderId: "1001952473982",
        appId: "1:1001952473982:web:a309b046972d3602d5b92f",
        measurementId: "G-X155LG29H6"
    };

    // Initialize Firebase
    const app = initializeApp(firebaseConfig);
    const auth = getAuth(app);

    const showPasswordCheckbox = document.getElementById('showPassword');
    const passwordInput = document.getElementById('password');

    showPasswordCheckbox.addEventListener('change', function() {
        if (showPasswordCheckbox.checked) {
            passwordInput.type = 'text';
        } else {
            passwordInput.type = 'password';
        }
    });

    // Handle Login Button Click
    const loginForm = document.getElementById('adminLoginForm');
    loginForm.addEventListener('submit', async (e) => {
        e.preventDefault();

        const email = document.getElementById('email').value;
        const password = document.getElementById('password').value;

        signInWithEmailAndPassword(auth, email, password)
            .then((userCredential) => {
                const user = userCredential.user;
                const uid = user.uid;
                $.ajax({
                    url: 'setSession.php',
                    type: 'POST',
                    data: {id: uid},
                    success:function(data){
                        location.href = "dashboard.php";
                    }
                });
            })
            .catch((error) => {
                alert('error: ' + error.message);
            });
    });
</script>

</body>
</html>

<?php include 'head.php'; ?>
<?php include 'navBar.php'; ?>



<section>
    <div class="container">
        <div class="row">
            <div class="col">
                <div class="container-fluid my-2">
                    <div class="d-flex justify-content-between mb-2">
                        <div class="d-flex align-items-end flex-grow-1">
                            <h2>Logs</h2>
                        </div>

                        <div class="mx-2">
                            <label for="hardHatSelect" class="form-label">Filter by Hard Hat ID</label>
                            <select id="hardHatSelect" class="form-select border border-success">
                                <option value="0">All Hard Hats</option>
                                <option value="1">Hard Hat 1</option>
                                <option value="2">Hard Hat 2</option>
                            </select>
                        </div>

                        <div class="mx-2">
                            <label for="sortSelect" class="form-label">Sort by Time</label>
                            <select id="sortSelect" class="form-select border border-success">
                                <option value="1">Oldest first</option>
                                <option value="2">Latest first</option>
                            </select>
                        </div>
                    </div>
                    <div class="table-responsive">
                        <table class="table table-bordered table-hover text-center">
                            <thead class="table-light">
                                <tr>
                                    <th>Timestamp</th>
                                    <th>ID</th>
                                    <th>Status</th>
                                    <th>Latitude</th>
                                    <th>Longitude</th>
                                </tr>
                            </thead>
                            <tbody id="logsTableBody">
                                <!-- Dashboard data rows will be rendered here -->
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script type="module">
        // Import the necessary Firebase SDKs
        import { initializeApp } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-app.js";
        import { getAnalytics } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-analytics.js";
        import { getDatabase, ref, onValue, push, set } from "https://www.gstatic.com/firebasejs/10.10.0/firebase-database.js";

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
        const analytics = getAnalytics(app);
        const db = getDatabase();

        const logsContent = document.getElementById("logsTableBody");

        const logsRef = ref(db, "logs");

        // Function to format Firebase timestamps
        function formatTimestamp(timestamp) {
            if (timestamp) {
                const date = new Date(timestamp);
                return date.toLocaleString(); // Formats to local time (adjust as needed)
            }
            return "Loading..."; // Return a placeholder if timestamp is not available
        }

        // Function to update the logs table content
        function updateLogsContent(data) {
            logsContent.innerHTML = ""; // Clear existing rows

            // Populate table rows with data
            data.forEach((log) => {
                const row = document.createElement("tr");
                row.innerHTML = `
                    <td>${formatTimestamp(log.timestamp)}</td>
                    <td>Hard Hat ${log.hardHatId}</td>
                    <td>${log.status ? "Active" : "Inactive"}</td>
                    <td>${log.latitude}</td>
                    <td>${log.longitude}</td>
                `;
                logsContent.appendChild(row); // Append the row to the table
            });
        }

        // Listen for changes in the "logs" node
        onValue(logsRef, (snapshot) => {
            const data = []; // Array to store retrieved data

            // Extract each child node value and include the timestamp as the parent key
            snapshot.forEach((childSnapshot) => {
                const log = childSnapshot.val();
                data.push(log);
            });

            console.log("Retrieved Data:", data); // Log the final data array for debugging

            const hardHatSelect = document.getElementById("hardHatSelect");
            const sortSelect = document.getElementById("sortSelect");

            // Function to filter and sort the data based on dropdown values
            function filterAndSortData() {
                let filteredData = data;

                // Filter by Hard Hat ID
                const selectedHardHatId = hardHatSelect.value;
                if (selectedHardHatId !== "0") {
                    filteredData = filteredData.filter(log => log.hardHatId === parseInt(selectedHardHatId));
                }

                // Sort by Time (ascending or descending)
                const sortOrder = sortSelect.value === "1" ? 1 : -1;
                filteredData.sort((a, b) => {
                    const timestampA = a.timestamp;
                    const timestampB = b.timestamp;
                    return (timestampA - timestampB) * sortOrder;
                });

                console.log("Filtered and Sorted Data:", filteredData); // Log the final data for debugging

                // Update the dashboard display with the filtered and sorted data
                updateLogsContent(filteredData);
            }

            // Attach event listeners to dropdowns to trigger filter and sort on change
            hardHatSelect.addEventListener('change', filterAndSortData);
            sortSelect.addEventListener('change', filterAndSortData);

            // Initially render all logs when the page loads
            filterAndSortData();
        });
    </script>
</section>

<?php include 'footer.php'; ?>

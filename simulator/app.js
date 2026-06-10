// app.js
// Main application coordinator for Docking Simulator UI

document.addEventListener('DOMContentLoaded', () => {
    // --- State Variables ---
    let engine = null;
    let isPlaying = false;
    let animationFrameId = null;
    let activeSpeed = 1; // 1 = Realtime (50ms interval), 2 = 2x, 5 = 5x, 0 = Max (requestAnimationFrame loop)
    let singleRunTimer = null;
    
    // Batch run state
    let activeBatchTrials = 100;
    let worker = null;
    let isBatchRunning = false;

    // --- DOM Elements ---
    const canvas = document.getElementById('arena');
    const ctx = canvas.getContext('2d');
    
    // Config Elements
    const algoSelect = document.getElementById('algo-select');
    const noiseToggle = document.getElementById('noise-toggle');
    
    const targetWInput = document.getElementById('target-w');
    const targetHInput = document.getElementById('target-h');
    const targetRotInput = document.getElementById('target-rot');
    
    // Geometry Displays
    const targetWVal = document.getElementById('target-w-val');
    const targetHVal = document.getElementById('target-h-val');
    const targetRotVal = document.getElementById('target-rot-val');
    
    // Noise Sliders
    const noisePosInput = document.getElementById('noise-pos');
    const noiseHeadingInput = document.getElementById('noise-heading');
    const noiseSlipInput = document.getElementById('noise-slip');
    const noiseTurnInput = document.getElementById('noise-turn-err');
    const noiseDelayInput = document.getElementById('noise-stop-delay');
    
    // Noise Displays
    const noisePosVal = document.getElementById('noise-pos-val');
    const noiseHeadingVal = document.getElementById('noise-heading-val');
    const noiseSlipVal = document.getElementById('noise-slip-val');
    const noiseTurnVal = document.getElementById('noise-turn-err-val');
    const noiseDelayVal = document.getElementById('noise-stop-delay-val');
    
    // Telemetry HUD Elements
    const hudTime = document.getElementById('hud-time');
    const hudX = document.getElementById('hud-x');
    const hudY = document.getElementById('hud-y');
    const hudHeading = document.getElementById('hud-heading');
    const hudDist = document.getElementById('hud-dist');
    const hudAngle = document.getElementById('hud-angle');
    const hudCoverage = document.getElementById('hud-coverage');
    const hudScore = document.getElementById('hud-score');
    const hudVisible = document.getElementById('hud-visible');
    const hudUnder = document.getElementById('hud-under');
    const hudAlert = document.getElementById('hud-alert');
    
    // Control Buttons
    const playBtn = document.getElementById('play-btn');
    const stepBtn = document.getElementById('step-btn');
    const resetBtn = document.getElementById('reset-btn');
    
    const speed1x = document.getElementById('speed-1x');
    const speed2x = document.getElementById('speed-2x');
    const speed5x = document.getElementById('speed-5x');
    const speedMax = document.getElementById('speed-max');
    
    // Batch Elements
    const trial100 = document.getElementById('trials-100');
    const trial1000 = document.getElementById('trials-1000');
    const trial10000 = document.getElementById('trials-10000');
    const runBatchBtn = document.getElementById('run-batch-btn');
    
    const progressContainer = document.getElementById('progress-container');
    const progressBarFill = document.getElementById('progress-bar-fill');
    const progressLabelText = document.getElementById('progress-label-text');
    
    // Stats Displays
    const statSuccess = document.getElementById('stat-success');
    const statPartial = document.getElementById('stat-partial');
    const statTouched = document.getElementById('stat-touched');
    const statMoving = document.getElementById('stat-moving');
    const statScore = document.getElementById('stat-score');
    const statTime = document.getElementById('stat-time');
    
    const logBox = document.getElementById('log-box');

    // --- Dynamic Slider Values ---
    function linkSlider(inputEl, displayEl, suffix = '') {
        inputEl.addEventListener('input', (e) => {
            displayEl.textContent = e.target.value + suffix;
        });
    }
    linkSlider(targetWInput, targetWVal, ' cm');
    linkSlider(targetHInput, targetHVal, ' cm');
    linkSlider(targetRotInput, targetRotVal, '°');
    linkSlider(noisePosInput, noisePosVal, ' cm');
    linkSlider(noiseHeadingInput, noiseHeadingVal, '°');
    linkSlider(noiseSlipInput, noiseSlipVal, '%');
    linkSlider(noiseTurnInput, noiseTurnVal, '%');
    linkSlider(noiseDelayInput, noiseDelayVal, ' s');

    // --- Helpers ---
    function addLog(text, type = 'info') {
        const entry = document.createElement('div');
        entry.className = `log-entry log-${type}`;
        
        const timestamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
        entry.textContent = `[${timestamp}] ${text}`;
        
        logBox.appendChild(entry);
        logBox.scrollTop = logBox.scrollHeight;
    }

    // Convert physics configs to Simulator format
    function getSimulationConfig() {
        return {
            target_width_cm: parseFloat(targetWInput.value),
            target_height_cm: parseFloat(targetHInput.value),
            target_rotation_deg: parseFloat(targetRotInput.value),
            
            noise_enabled: noiseToggle.checked,
            position_noise_cm: parseFloat(noisePosInput.value),
            heading_noise_deg: parseFloat(noiseHeadingInput.value),
            forward_slip_percent: parseFloat(noiseSlipInput.value),
            turn_error_percent: parseFloat(noiseTurnInput.value),
            stop_delay_s: parseFloat(noiseDelayInput.value),
            
            // Standard constraints
            forward_speed_cm_s: 10,
            turn_speed_deg_s: 45,
            time_step_s: 0.05,
            start_distance_min_cm: 100,
            start_distance_max_cm: 130
        };
    }

    // Initialize Simulator Engine
    function initSimulation() {
        const config = getSimulationConfig();
        engine = new Simulator.Engine(config);
        isPlaying = false;
        playBtn.innerHTML = '▶ Play';
        playBtn.className = 'btn btn-success';
        
        if (singleRunTimer) clearInterval(singleRunTimer);
        if (animationFrameId) cancelAnimationFrame(animationFrameId);
        
        updateTelemetry();
        renderArena();
        addLog(`Simulator initialized. Start pose: (${engine.x.toFixed(1)}, ${engine.y.toFixed(1)}, ${engine.heading.toFixed(0)}°)`, 'info');
    }

    // Scale mapping (physics centimeters to Canvas pixels)
    // Canvas: 600x600 pixels. Centered at (300, 300).
    // Covering a 300x300 cm area (-150 to +150 cm).
    // 600px / 300cm = 2.0 pixels per cm.
    const SCALE = 2.0; 
    const OFFSET_X = 300;
    const OFFSET_Y = 300;

    function toCanvasX(xCm) {
        return OFFSET_X + xCm * SCALE;
    }
    
    function toCanvasY(yCm) {
        // Invert Y axis so +Y goes upwards in physics view
        return OFFSET_Y - yCm * SCALE;
    }

    // --- Renderer ---
    function renderArena() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // 1. Draw Grid Lines (every 10 cm)
        ctx.strokeStyle = '#111827';
        ctx.lineWidth = 1;
        
        // Vertical lines
        for (let x = -150; x <= 150; x += 10) {
            ctx.beginPath();
            ctx.moveTo(toCanvasX(x), toCanvasY(-150));
            ctx.lineTo(toCanvasX(x), toCanvasY(150));
            ctx.stroke();
        }
        
        // Horizontal lines
        for (let y = -150; y <= 150; y += 10) {
            ctx.beginPath();
            ctx.moveTo(toCanvasX(-150), toCanvasY(y));
            ctx.lineTo(toCanvasX(150), toCanvasY(y));
            ctx.stroke();
        }
        
        // Major Axes (Origin)
        ctx.strokeStyle = '#1f2937';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(toCanvasX(-150), toCanvasY(0));
        ctx.lineTo(toCanvasX(150), toCanvasY(0));
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(toCanvasX(0), toCanvasY(-150));
        ctx.lineTo(toCanvasX(0), toCanvasY(150));
        ctx.stroke();
        
        if (!engine) return;
        
        const sensors = engine.getSensors();
        
        // 2. Draw Target Mat
        const targetPoly = engine.getTargetPolygon();
        ctx.fillStyle = 'rgba(245, 158, 11, 0.15)'; // transparent amber
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(toCanvasX(targetPoly[0].x), toCanvasY(targetPoly[0].y));
        for (let i = 1; i < targetPoly.length; i++) {
            ctx.lineTo(toCanvasX(targetPoly[i].x), toCanvasY(targetPoly[i].y));
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        
        // 3. Draw Robot Trajectory Path
        if (engine.path.length > 1) {
            ctx.strokeStyle = 'rgba(99, 102, 241, 0.4)'; // transparent indigo
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(toCanvasX(engine.path[0].x), toCanvasY(engine.path[0].y));
            for (let i = 1; i < engine.path.length; i++) {
                ctx.lineTo(toCanvasX(engine.path[i].x), toCanvasY(engine.path[i].y));
            }
            ctx.stroke();
        }
        
        // 4. Draw Sensor Cone (FOV)
        const sensorRange = engine.config.sensor_range_cm;
        const sensorFov = engine.config.sensor_fov_deg;
        const headingRad = engine.heading * Math.PI / 180;
        
        ctx.fillStyle = sensors.target_visible ? 'rgba(16, 185, 129, 0.08)' : 'rgba(243, 244, 246, 0.03)';
        ctx.strokeStyle = sensors.target_visible ? 'rgba(16, 185, 129, 0.2)' : 'rgba(243, 244, 246, 0.08)';
        ctx.lineWidth = 1;
        
        ctx.beginPath();
        ctx.moveTo(toCanvasX(engine.x), toCanvasY(engine.y));
        const fovHalfRad = (sensorFov / 2) * Math.PI / 180;
        
        // Draw sensor arc (in physics coordinate space Y goes up, so angles are inverted in canvas coordinates)
        const canvasHeading = -headingRad; // negate for canvas coords
        const startAngle = canvasHeading - fovHalfRad;
        const endAngle = canvasHeading + fovHalfRad;
        
        ctx.arc(
            toCanvasX(engine.x), 
            toCanvasY(engine.y), 
            sensorRange * SCALE, 
            startAngle, 
            endAngle, 
            false
        );
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        
        // Draw bearing line to target center if visible
        if (sensors.target_visible) {
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.moveTo(toCanvasX(engine.x), toCanvasY(engine.y));
            ctx.lineTo(toCanvasX(engine.target_x), toCanvasY(engine.target_y));
            ctx.stroke();
            ctx.setLineDash([]); // Reset
        }
        
        // 5. Draw Polygon Intersection (Clipped Coverage Area)
        const robotPoly = engine.getRobotPolygon();
        const intersectPoly = Simulator.clipPolygon(robotPoly, targetPoly);
        if (intersectPoly.length >= 3) {
            ctx.fillStyle = 'rgba(16, 185, 129, 0.4)'; // bright semi-transparent emerald
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(toCanvasX(intersectPoly[0].x), toCanvasY(intersectPoly[0].y));
            for (let i = 1; i < intersectPoly.length; i++) {
                ctx.lineTo(toCanvasX(intersectPoly[i].x), toCanvasY(intersectPoly[i].y));
            }
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
        }
        
        // 6. Draw Robot Footprint Chamfered Polygon
        ctx.fillStyle = 'rgba(99, 102, 241, 0.2)'; // indigo body
        ctx.strokeStyle = '#6366f1';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(toCanvasX(robotPoly[0].x), toCanvasY(robotPoly[0].y));
        for (let i = 1; i < robotPoly.length; i++) {
            ctx.lineTo(toCanvasX(robotPoly[i].x), toCanvasY(robotPoly[i].y));
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        
        // 7. Draw Robot Heading Indicator (Forward Arrow Face)
        const noseLength = 6.0; // cm
        const noseX = engine.x + (engine.config.robot_length_cm / 2 + noseLength) * Math.cos(headingRad);
        const noseY = engine.y + (engine.config.robot_length_cm / 2 + noseLength) * Math.sin(headingRad);
        
        ctx.fillStyle = '#6366f1';
        ctx.beginPath();
        ctx.arc(toCanvasX(noseX), toCanvasY(noseY), 4, 0, 2*Math.PI);
        ctx.fill();
        
        // Center pin
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(toCanvasX(engine.x), toCanvasY(engine.y), 3, 0, 2*Math.PI);
        ctx.fill();
    }

    // Update Telemetry HUD displays
    function updateTelemetry() {
        if (!engine) return;
        
        const sensors = engine.getSensors();
        const scoreObj = engine.getScore();
        
        hudTime.textContent = engine.time.toFixed(2);
        hudX.textContent = engine.x.toFixed(1);
        hudY.textContent = engine.y.toFixed(1);
        hudHeading.textContent = engine.heading.toFixed(0);
        
        hudDist.textContent = sensors.target_visible ? sensors.target_distance_cm.toFixed(1) : '---';
        hudAngle.textContent = sensors.target_visible ? sensors.target_angle_deg.toFixed(1) : '---';
        
        hudCoverage.textContent = (sensors.coverage_ratio * 100).toFixed(1);
        hudScore.textContent = scoreObj.score;
        
        hudVisible.textContent = sensors.target_visible ? 'YES' : 'NO';
        hudVisible.style.color = sensors.target_visible ? 'var(--success)' : 'var(--text-muted)';
        
        hudUnder.textContent = sensors.target_under_robot ? 'YES' : 'NO';
        hudUnder.style.color = sensors.target_under_robot ? 'var(--success)' : 'var(--text-muted)';
        
        // Handle indicator alert
        if (engine.termination_triggered) {
            hudAlert.textContent = "TERMINATED";
            hudAlert.style.backgroundColor = "rgba(16, 185, 129, 0.15)";
            hudAlert.style.borderColor = "var(--success)";
            hudAlert.style.color = "var(--success)";
            hudAlert.classList.add('active');
        } else if (engine.indicator_triggered_while_moving) {
            hudAlert.textContent = "TERMINATION PENALTY";
            hudAlert.style.backgroundColor = "rgba(239, 68, 68, 0.15)";
            hudAlert.style.borderColor = "var(--danger)";
            hudAlert.style.color = "var(--danger)";
            hudAlert.classList.add('active');
        } else {
            hudAlert.classList.remove('active');
        }
    }

    // --- Simulation Loop Manager ---
    function performStep() {
        if (!engine || engine.terminated) {
            if (engine && engine.terminated) {
                stopSimulation();
                const scoreObj = engine.getScore();
                let detail = `Completed with score: ${scoreObj.score} (Coverage: ${(scoreObj.coverage_ratio * 100).toFixed(1)}%) in ${scoreObj.time_s.toFixed(2)}s.`;
                if (scoreObj.moving_failure) {
                    detail += ` [Penalized: Indicator triggered while moving]`;
                }
                
                const typeMap = {
                    'full': 'success',
                    'partial': 'warning',
                    'touched': 'info',
                    'moving_termination': 'danger',
                    'failed': 'danger'
                };
                
                addLog(`Docking run finished. ${detail}`, typeMap[scoreObj.result_type] || 'info');
            }
            return;
        }
        
        const dt = engine.config.time_step_s;
        const sensors = engine.getSensors();
        const action = Simulator.getControllerAction(
            algoSelect.value,
            sensors,
            dt,
            engine.algoState,
            engine.heading
        );
        
        engine.step(action);
        
        updateTelemetry();
        renderArena();
    }

    function startSimulation() {
        isPlaying = true;
        playBtn.innerHTML = '⏸ Pause';
        playBtn.className = 'btn btn-primary';
        
        addLog(`Simulation started. Algorithm: ${algoSelect.value}`, 'info');
        
        if (activeSpeed === 0) {
            // Max Speed: requestAnimationFrame loop executing multiple steps per frame
            const maxSpeedLoop = () => {
                if (!isPlaying) return;
                // Run 20 steps (1.0 second of simulation time) per frame for max speed
                for (let step = 0; step < 20; step++) {
                    if (engine && !engine.terminated) {
                        performStep();
                    } else {
                        break;
                    }
                }
                if (engine && !engine.terminated) {
                    animationFrameId = requestAnimationFrame(maxSpeedLoop);
                } else {
                    performStep(); // Final step evaluation log
                }
            };
            animationFrameId = requestAnimationFrame(maxSpeedLoop);
        } else {
            // Clock-timer based speed execution
            // 1x = step every 50ms (sim dt=50ms)
            // 2x = step every 25ms
            // 5x = step every 10ms
            const interval = 50 / activeSpeed;
            singleRunTimer = setInterval(() => {
                performStep();
            }, interval);
        }
    }

    function stopSimulation() {
        isPlaying = false;
        playBtn.innerHTML = '▶ Play';
        playBtn.className = 'btn btn-success';
        
        if (singleRunTimer) clearInterval(singleRunTimer);
        if (animationFrameId) cancelAnimationFrame(animationFrameId);
    }

    // --- Interactive Control Listeners ---
    playBtn.addEventListener('click', () => {
        if (isPlaying) {
            stopSimulation();
            addLog('Simulation paused.', 'info');
        } else {
            startSimulation();
        }
    });

    stepBtn.addEventListener('click', () => {
        stopSimulation();
        performStep();
        addLog('Step executed.', 'info');
    });

    resetBtn.addEventListener('click', () => {
        stopSimulation();
        initSimulation();
    });

    // Speed selector buttons
    function setSpeed(speedVal, btnElement) {
        activeSpeed = speedVal;
        
        [speed1x, speed2x, speed5x, speedMax].forEach(btn => btn.classList.remove('active'));
        btnElement.classList.add('active');
        
        if (isPlaying) {
            stopSimulation();
            startSimulation();
        }
    }
    
    speed1x.addEventListener('click', () => setSpeed(1, speed1x));
    speed2x.addEventListener('click', () => setSpeed(2, speed2x));
    speed5x.addEventListener('click', () => setSpeed(5, speed5x));
    speedMax.addEventListener('click', () => setSpeed(0, speedMax));

    // Reset simulator if config changes
    [targetWInput, targetHInput, targetRotInput, noiseToggle].forEach(el => {
        el.addEventListener('change', () => {
            initSimulation();
        });
    });

    // --- Batch Simulation Run ---
    
    // Select batch size option card
    function selectBatchSize(size, cardEl) {
        activeBatchTrials = size;
        [trial100, trial1000, trial10000].forEach(card => card.classList.remove('active'));
        cardEl.classList.add('active');
    }
    
    trial100.addEventListener('click', () => selectBatchSize(100, trial100));
    trial1000.addEventListener('click', () => selectBatchSize(1000, trial1000));
    trial10000.addEventListener('click', () => selectBatchSize(10000, trial10000));

    // Display statistics metrics in the UI cards
    function updateBatchStats(stats) {
        statSuccess.textContent = `${stats.success_rate_full_coverage.toFixed(1)}%`;
        statPartial.textContent = `${stats.partial_coverage_rate.toFixed(1)}%`;
        statTouched.textContent = `${stats.touched_but_failed_rate.toFixed(1)}%`;
        statMoving.textContent = `${stats.moving_termination_failures.toFixed(1)}%`;
        
        statScore.textContent = stats.average_score.toFixed(2);
        statTime.textContent = `${stats.average_time_s.toFixed(1)}s`;
    }

    // Fallback: Run Batch simulation asynchronously in chunks on main thread (in case of local CORS web worker restriction)
    function runBatchOnMainThread(algoName, trialsCount, config) {
        addLog('Web Worker blocked or unavailable. Falling back to Main Thread async chunking batch run.', 'warning');
        
        let fullCoverageCount = 0;
        let partialCoverageCount = 0;
        let touchedButFailedCount = 0;
        let movingTerminationFailures = 0;
        let totalScore = 0;
        let totalTime = 0;
        
        let completed = 0;
        const dt = config.time_step_s || 0.05;
        const chunkSize = 20; // run 20 trials per asynchronous tick
        
        function processChunk() {
            if (!isBatchRunning) return; // run cancelled
            
            const targetEnd = Math.min(completed + chunkSize, trialsCount);
            
            for (let i = completed; i < targetEnd; i++) {
                const bEngine = new Simulator.Engine(config);
                while (!bEngine.terminated) {
                    const sensors = bEngine.getSensors();
                    const action = Simulator.getControllerAction(
                        algoName,
                        sensors,
                        dt,
                        bEngine.algoState,
                        bEngine.heading
                    );
                    bEngine.step(action);
                }
                
                const result = bEngine.getScore();
                totalScore += result.score;
                totalTime += result.time_s;
                
                if (result.moving_failure) {
                    movingTerminationFailures++;
                }
                
                switch (result.result_type) {
                    case 'full':
                        fullCoverageCount++;
                        break;
                    case 'partial':
                        partialCoverageCount++;
                        break;
                    case 'touched':
                        touchedButFailedCount++;
                        break;
                }
            }
            
            completed = targetEnd;
            const progressPct = Math.round((completed / trialsCount) * 100);
            
            progressBarFill.style.width = `${progressPct}%`;
            progressLabelText.textContent = `Completed ${completed} / ${trialsCount} trials`;
            
            // Calculate intermediate stats
            const tempStats = {
                success_rate_full_coverage: (fullCoverageCount / completed) * 100,
                partial_coverage_rate: (partialCoverageCount / completed) * 100,
                touched_but_failed_rate: (touchedButFailedCount / completed) * 100,
                moving_termination_failures: (movingTerminationFailures / completed) * 100,
                average_score: totalScore / completed,
                average_time_s: totalTime / completed
            };
            
            updateBatchStats(tempStats);
            
            if (completed < trialsCount) {
                // Yield to event loop to prevent UI blocking
                setTimeout(processChunk, 0);
            } else {
                // Done!
                isBatchRunning = false;
                runBatchBtn.textContent = '🚀 Run Batch Simulation';
                runBatchBtn.classList.remove('btn-primary');
                runBatchBtn.classList.add('btn-success');
                progressContainer.style.display = 'none';
                
                addLog(`Batch Simulation complete! Trials: ${trialsCount}, Avg Score: ${tempStats.average_score.toFixed(2)}, Success rate: ${tempStats.success_rate_full_coverage.toFixed(1)}%`, 'success');
            }
        }
        
        // Start process loop
        setTimeout(processChunk, 0);
    }

    // Main Run Batch trigger
    runBatchBtn.addEventListener('click', () => {
        if (isBatchRunning) {
            // Cancel running batch
            isBatchRunning = false;
            if (worker) {
                worker.terminate();
                worker = null;
            }
            runBatchBtn.textContent = '🚀 Run Batch Simulation';
            runBatchBtn.classList.remove('btn-primary');
            runBatchBtn.classList.add('btn-success');
            progressContainer.style.display = 'none';
            addLog('Batch run cancelled by user.', 'warning');
            return;
        }
        
        isBatchRunning = true;
        runBatchBtn.textContent = '🛑 Cancel Batch Run';
        runBatchBtn.classList.remove('btn-success');
        runBatchBtn.classList.add('btn-primary');
        
        progressContainer.style.display = 'flex';
        progressBarFill.style.width = '0%';
        progressLabelText.textContent = `Initializing batch of ${activeBatchTrials} trials...`;
        
        const algoName = algoSelect.value;
        const config = getSimulationConfig();
        
        addLog(`Starting batch simulation: ${activeBatchTrials} trials using algorithm: ${algoName}...`, 'info');
        
        // Attempt web worker execution
        try {
            // Use standard Worker constructor.
            // If running on local file:// protocol, this might throw a security error or fail to load.
            worker = new Worker('worker.js');
            
            worker.onmessage = function(e) {
                if (!isBatchRunning) return;
                
                const data = e.data;
                if (data.type === 'progress') {
                    progressBarFill.style.width = `${data.progressPercent}%`;
                    progressLabelText.textContent = `Completed ${data.trialsCompleted} / ${activeBatchTrials} trials`;
                    updateBatchStats(data.stats);
                } else if (data.type === 'complete') {
                    isBatchRunning = false;
                    runBatchBtn.textContent = '🚀 Run Batch Simulation';
                    runBatchBtn.classList.remove('btn-primary');
                    runBatchBtn.classList.add('btn-success');
                    progressContainer.style.display = 'none';
                    
                    updateBatchStats(data.stats);
                    addLog(`Batch Simulation complete! Trials: ${data.totalTrials}, Avg Score: ${data.stats.average_score.toFixed(2)}, Success rate: ${data.stats.success_rate_full_coverage.toFixed(1)}%`, 'success');
                    
                    worker.terminate();
                    worker = null;
                }
            };
            
            worker.onerror = function(err) {
                // If worker throws error or fails to load, fallback to main thread async chunks
                console.error("Worker error: ", err);
                if (worker) {
                    worker.terminate();
                    worker = null;
                }
                runBatchOnMainThread(algoName, activeBatchTrials, config);
            };
            
            // Start the worker processing
            worker.postMessage({
                algoName,
                trialsCount: activeBatchTrials,
                config
            });
            
        } catch (error) {
            // Catch synchronous constructor errors (like file:// SecurityError in Chrome/Safari)
            console.warn("Caught worker constructor exception, falling back to main thread:", error);
            runBatchOnMainThread(algoName, activeBatchTrials, config);
        }
    });

    // Initialize UI on load
    initSimulation();
});

// simulator.js
// Docking Simulator Engine - Independent of Rendering

const Simulator = (function() {
    // --- Math Helpers ---

    // Box-Muller transform for generating zero-mean, unit-variance Gaussian random variables
    function randomGaussian() {
        let u = 0, v = 0;
        while(u === 0) u = Math.random(); // Converting [0,1) to (0,1)
        while(v === 0) v = Math.random();
        return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    }

    // Force a convex polygon's vertices into Counter-Clockwise (CCW) order
    function makeCCW(polygon) {
        let sum = 0;
        for (let i = 0; i < polygon.length; i++) {
            const p1 = polygon[i];
            const p2 = polygon[(i + 1) % polygon.length];
            sum += (p2.x - p1.x) * (p2.y + p1.y);
        }
        if (sum > 0) {
            return [...polygon].reverse();
        }
        return polygon;
    }

    // Calculate the area of a polygon using the Shoelace formula
    function getPolygonArea(vertices) {
        let area = 0;
        const n = vertices.length;
        if (n < 3) return 0;
        for (let i = 0; i < n; i++) {
            const j = (i + 1) % n;
            area += vertices[i].x * vertices[j].y;
            area -= vertices[j].x * vertices[i].y;
        }
        return Math.abs(area) / 2.0;
    }

    // Sutherland-Hodgman Polygon Clipping (finds intersection of two convex polygons)
    function clipPolygon(subjectPolygon, clipPolygon) {
        const sub = makeCCW(subjectPolygon);
        const clip = makeCCW(clipPolygon);
        
        let outputList = sub;
        
        for (let i = 0; i < clip.length; i++) {
            const edgeStart = clip[i];
            const edgeEnd = clip[(i + 1) % clip.length];
            
            const inputList = outputList;
            outputList = [];
            
            if (inputList.length === 0) break;
            
            let s = inputList[inputList.length - 1];
            
            for (let j = 0; j < inputList.length; j++) {
                const p = inputList[j];
                
                if (isLeftOfEdge(p, edgeStart, edgeEnd)) {
                    if (!isLeftOfEdge(s, edgeStart, edgeEnd)) {
                        outputList.push(computeIntersection(s, p, edgeStart, edgeEnd));
                    }
                    outputList.push(p);
                } else if (isLeftOfEdge(s, edgeStart, edgeEnd)) {
                    outputList.push(computeIntersection(s, p, edgeStart, edgeEnd));
                }
                s = p;
            }
        }
        return outputList;
    }

    // Helper: is point p to the left of directed edge edgeStart -> edgeEnd? (CCW interior)
    function isLeftOfEdge(p, edgeStart, edgeEnd) {
        return (edgeEnd.x - edgeStart.x) * (p.y - edgeStart.y) - (edgeEnd.y - edgeStart.y) * (p.x - edgeStart.x) >= -1e-9;
    }

    // Helper: compute intersection of segment s-p with line edgeStart-edgeEnd
    function computeIntersection(s, p, edgeStart, edgeEnd) {
        const dc = { x: edgeStart.x - edgeEnd.x, y: edgeStart.y - edgeEnd.y };
        const dp = { x: s.x - p.x, y: s.y - p.y };
        const n1 = edgeStart.x * edgeEnd.y - edgeStart.y * edgeEnd.x;
        const n2 = s.x * p.y - s.y * p.x;
        const den = dc.x * dp.y - dc.y * dp.x;
        
        if (Math.abs(den) < 1e-9) {
            return { x: s.x, y: s.y }; 
        }
        return {
            x: (n1 * dp.x - dc.x * n2) / den,
            y: (n1 * dp.y - dc.y * n2) / den
        };
    }

    // Check if point p is inside convex CCW polygon
    function isPointInConvexPolygon(p, polygon) {
        const ccwPoly = makeCCW(polygon);
        for (let i = 0; i < ccwPoly.length; i++) {
            const edgeStart = ccwPoly[i];
            const edgeEnd = ccwPoly[(i + 1) % ccwPoly.length];
            if ((edgeEnd.x - edgeStart.x) * (p.y - edgeStart.y) - (edgeEnd.y - edgeStart.y) * (p.x - edgeStart.x) < -1e-9) {
                return false;
            }
        }
        return true;
    }

    // Normalize angle to [-180, 180] degrees
    function normalizeAngle(angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // --- Robot Local Footprint ---
    // Robot points oriented facing along +X axis (length=16cm, width=12cm)
    const ROBOT_LOCAL_POINTS = [
        { x: 8.0, y: -3.5 },  // Front-right chamfer
        { x: 8.0, y: 3.5 },   // Front-left chamfer
        { x: 5.5, y: 6.0 },   // Front-left corner
        { x: -5.5, y: 6.0 },  // Back-left corner
        { x: -8.0, y: 3.5 },  // Back-left chamfer
        { x: -8.0, y: -3.5 }, // Back-right chamfer
        { x: -5.5, y: -6.0 }, // Back-right corner
        { x: 5.5, y: -6.0 }   // Front-right corner
    ];

    // --- Simulator Engine Class ---
    class Engine {
        constructor(config = {}) {
            this.config = {
                // Robot specs
                robot_width_cm: 12,
                robot_length_cm: 16,
                
                // Target specs
                target_width_cm: 5,
                target_height_cm: 5,
                target_rotation_deg: 0,
                
                // Motion parameters
                forward_speed_cm_s: 10,
                turn_speed_deg_s: 45,
                time_step_s: 0.05, // 50ms default
                
                // Start conditions
                start_distance_min_cm: 100,
                start_distance_max_cm: 130,
                start_position_random: true,
                robot_orientation_random: true,
                
                // Noise controls
                noise_enabled: false,
                position_noise_cm: 0.1,       // drift per step
                heading_noise_deg: 0.5,       // drift per step
                forward_slip_percent: 10,     // slip reduction up to 10%
                turn_error_percent: 10,       // rotational variance (stddev as % of command)
                stop_delay_s: 0.1,            // delay in stopping
                
                // Sensor specs & noise
                sensor_fov_deg: 60,           // total FOV angle
                sensor_range_cm: 150,         // max visibility range
                sensor_false_positive_rate: 0.01,
                sensor_false_negative_rate: 0.02,
                sensor_range_noise_cm: 1.0,   // stddev of distance
                sensor_angle_noise_deg: 2.0,  // stddev of relative angle
                
                // Sim settings
                max_simulation_time_s: 180,   // 3 minutes timeout
                
                ...config
            };

            this.reset();
        }

        reset() {
            this.time = 0;
            this.path = [];
            this.terminated = false;
            this.termination_triggered = false;
            
            // Motion state
            this.x = 0;
            this.y = 0;
            this.heading = 0; // degrees, 0 is along +X
            
            // Current command inputs
            this.current_v_cmd = 0;     // cm/s
            this.current_omega_cmd = 0; // deg/s
            
            // Target definition
            this.target_x = 0;
            this.target_y = 0;
            this.target_theta = this.config.target_rotation_deg;
            
            // Metrics tracker
            this.touched_target = false;
            this.indicator_triggered_while_moving = false;
            this.stop_delay_timer = 0;
            
            // Initialize start pose
            let dist = this.config.start_distance_min_cm;
            if (this.config.start_position_random) {
                const range = this.config.start_distance_max_cm - this.config.start_distance_min_cm;
                dist = this.config.start_distance_min_cm + Math.random() * Math.max(0, range);
            }
            
            let angle_rad = 0;
            if (this.config.start_position_random) {
                angle_rad = Math.random() * 2 * Math.PI;
            }
            
            this.x = dist * Math.cos(angle_rad);
            this.y = dist * Math.sin(angle_rad);
            
            if (this.config.robot_orientation_random) {
                this.heading = normalizeAngle(Math.random() * 360);
            } else {
                // Face the target (approximate direction)
                const to_target = Math.atan2(this.target_y - this.y, this.target_x - this.x) * 180 / Math.PI;
                this.heading = normalizeAngle(to_target);
            }
            
            this.path.push({ x: this.x, y: this.y });
            
            // Reset algorithm state if it has any
            this.algoState = {};
        }

        // Get global coordinates of the robot's vertices
        getRobotPolygon() {
            const rad = this.heading * Math.PI / 180;
            const cos = Math.cos(rad);
            const sin = Math.sin(rad);
            return ROBOT_LOCAL_POINTS.map(p => ({
                x: this.x + p.x * cos - p.y * sin,
                y: this.y + p.x * sin + p.y * cos
            }));
        }

        // Get global coordinates of the target mat
        getTargetPolygon() {
            const w = this.config.target_width_cm;
            const h = this.config.target_height_cm;
            const rad = this.target_theta * Math.PI / 180;
            const cos = Math.cos(rad);
            const sin = Math.sin(rad);
            
            const localTarget = [
                { x: -w/2, y: -h/2 },
                { x: w/2, y: -h/2 },
                { x: w/2, y: h/2 },
                { x: -w/2, y: h/2 }
            ];
            
            return localTarget.map(p => ({
                x: this.target_x + p.x * cos - p.y * sin,
                y: this.target_y + p.x * sin + p.y * cos
            }));
        }

        // Calculate geometric coverage ratio
        getCoverageRatio() {
            const robotPoly = this.getRobotPolygon();
            const targetPoly = this.getTargetPolygon();
            
            const intersectionPoly = clipPolygon(robotPoly, targetPoly);
            const interArea = getPolygonArea(intersectionPoly);
            const targetArea = this.config.target_width_cm * this.config.target_height_cm;
            
            return Math.min(1.0, Math.max(0.0, interArea / targetArea));
        }

        // Checks if target center is within the robot's footprint
        isTargetUnderRobot() {
            const robotPoly = this.getRobotPolygon();
            const center = { x: this.target_x, y: this.target_y };
            return isPointInConvexPolygon(center, robotPoly);
        }

        // Return clean virtual sensors
        getSensors() {
            const dx = this.target_x - this.x;
            const dy = this.target_y - this.y;
            const rawDist = Math.sqrt(dx*dx + dy*dy);
            
            // Relative angle from robot's heading to target center
            const globalAngleToTarget = Math.atan2(dy, dx) * 180 / Math.PI;
            const rawRelativeAngle = normalizeAngle(globalAngleToTarget - this.heading);
            
            // Base visibility condition (sensor cone)
            const halfFov = this.config.sensor_fov_deg / 2;
            let rawVisible = (rawDist <= this.config.sensor_range_cm) && 
                             (Math.abs(rawRelativeAngle) <= halfFov);
            
            let visible = rawVisible;
            let dist = rawDist;
            let relAngle = rawRelativeAngle;
            
            // Apply noise if enabled
            if (this.config.noise_enabled) {
                // False negative
                if (rawVisible && Math.random() < this.config.sensor_false_negative_rate) {
                    visible = false;
                }
                // False positive
                if (!rawVisible && Math.random() < this.config.sensor_false_positive_rate) {
                    visible = true;
                }
                
                // Distance noise (only if visible, or sometimes random if false positive)
                if (visible) {
                    dist = Math.max(0, rawDist + randomGaussian() * this.config.sensor_range_noise_cm);
                    relAngle = normalizeAngle(rawRelativeAngle + randomGaussian() * this.config.sensor_angle_noise_deg);
                } else {
                    dist = 0;
                    relAngle = 0;
                }
            }
            
            return {
                target_visible: visible,
                target_angle_deg: relAngle,
                target_distance_cm: dist,
                target_under_robot: this.isTargetUnderRobot(),
                coverage_ratio: this.getCoverageRatio()
            };
        }

        // Updates physical simulation by one time-step
        step(action) {
            if (this.terminated) return;
            
            const dt = this.config.time_step_s;
            this.time += dt;
            
            // Check timeout
            if (this.time >= this.config.max_simulation_time_s) {
                this.terminated = true;
                return;
            }
            
            // Set speed command based on action
            let target_v = 0;
            let target_omega = 0;
            
            switch (action.command) {
                case 'move_forward':
                    target_v = this.config.forward_speed_cm_s;
                    target_omega = 0;
                    break;
                case 'turn_left':
                    target_v = 0;
                    target_omega = this.config.turn_speed_deg_s;
                    break;
                case 'turn_right':
                    target_v = 0;
                    target_omega = -this.config.turn_speed_deg_s;
                    break;
                case 'stop':
                default:
                    target_v = 0;
                    target_omega = 0;
                    break;
            }
            
            // Handle Stop Delay noise: if we command a stop, it takes a short delay before speeds drop to 0
            if (this.config.noise_enabled && target_v === 0 && target_omega === 0 && (this.current_v_cmd !== 0 || this.current_omega_cmd !== 0)) {
                if (this.stop_delay_timer === 0) {
                    this.stop_delay_timer = this.config.stop_delay_s;
                }
            }
            
            if (this.stop_delay_timer > 0) {
                this.stop_delay_timer -= dt;
                // keep running current command instead of stopping
                if (this.stop_delay_timer <= 0) {
                    this.stop_delay_timer = 0;
                    this.current_v_cmd = 0;
                    this.current_omega_cmd = 0;
                }
            } else {
                this.current_v_cmd = target_v;
                this.current_omega_cmd = target_omega;
            }
            
            // Handle Action Termination Indicator
            if (action.trigger_termination && !this.termination_triggered) {
                this.termination_triggered = true;
                
                // If robot is moving (either commanded or sliding) when termination is triggered, mark penalty
                const isMoving = this.current_v_cmd !== 0 || this.current_omega_cmd !== 0;
                if (isMoving) {
                    this.indicator_triggered_while_moving = true;
                }
            }
            
            // Apply velocities with noise
            let active_v = this.current_v_cmd;
            let active_omega = this.current_omega_cmd;
            
            if (this.config.noise_enabled) {
                // Forward Slip: reduction in velocity
                if (active_v > 0) {
                    const slipVal = Math.random() * (this.config.forward_slip_percent / 100);
                    active_v = active_v * (1 - slipVal);
                }
                
                // Turn Error: perturbation in angular speed
                if (active_omega !== 0) {
                    const percentError = 1 + (randomGaussian() * (this.config.turn_error_percent / 100));
                    active_omega = active_omega * percentError;
                }
            }
            
            // Integrate pose
            const rad = this.heading * Math.PI / 180;
            let dx = active_v * Math.cos(rad) * dt;
            let dy = active_v * Math.sin(rad) * dt;
            let dheading = active_omega * dt;
            
            // Apply drift noise per step
            if (this.config.noise_enabled) {
                dx += randomGaussian() * this.config.position_noise_cm;
                dy += randomGaussian() * this.config.position_noise_cm;
                dheading += randomGaussian() * this.config.heading_noise_deg;
            }
            
            this.x += dx;
            this.y += dy;
            this.heading = normalizeAngle(this.heading + dheading);
            
            this.path.push({ x: this.x, y: this.y });
            
            // Track if we touch target at any point during run
            const currentCoverage = this.getCoverageRatio();
            if (currentCoverage > 0.0) {
                this.touched_target = true;
            }
            
            // Evaluate termination state:
            // Terminated if:
            // 1. Termination indicator was triggered AND robot is stationary (i.e. target commands are 0 and delay timer is inactive)
            // 2. Timeout has been reached (already handled at top of step)
            if (this.termination_triggered) {
                const isStationary = this.current_v_cmd === 0 && this.current_omega_cmd === 0 && this.stop_delay_timer === 0;
                if (isStationary) {
                    this.terminated = true;
                }
            }
        }

        // Get final scores and metrics
        getScore() {
            const coverage = this.getCoverageRatio();
            
            let baseScore = 0;
            let resultType = 'failed';
            
            if (this.indicator_triggered_while_moving) {
                baseScore = -5;
                resultType = 'moving_termination';
            } else if (coverage >= 0.999) {
                baseScore = 10;
                resultType = 'full';
            } else if (coverage > 0) {
                baseScore = 2.5;
                resultType = 'partial';
            } else if (this.touched_target) {
                baseScore = 1;
                resultType = 'touched';
            } else {
                baseScore = 0;
                resultType = 'failed';
            }
            
            return {
                score: baseScore,
                coverage_ratio: coverage,
                result_type: resultType,
                time_s: this.time,
                touched: this.touched_target,
                moving_failure: this.indicator_triggered_while_moving
            };
        }
    }

    // --- Docking Controllers ---

    // 1. Straight Walk Controller
    function runStraightWalk(sensors, dt, algoState) {
        if (!algoState.mode) {
            algoState.mode = 'drive';
            algoState.timer = 0;
        }

        if (algoState.mode === 'drive') {
            // Just walk straight. Transition to stop-and-wait if target is under robot.
            if (sensors.target_under_robot) {
                algoState.mode = 'stop_and_wait';
                algoState.timer = 0;
                return {
                    command: 'stop',
                    trigger_termination: false
                };
            }
            return {
                command: 'move_forward',
                trigger_termination: false
            };
        } else if (algoState.mode === 'stop_and_wait') {
            algoState.timer += dt;
            // Wait 0.5 seconds to ensure we are completely stationary
            if (algoState.timer >= 0.5) {
                return {
                    command: 'stop',
                    trigger_termination: true
                };
            }
            return {
                command: 'stop',
                trigger_termination: false
            };
        }
        
        return { command: 'stop', trigger_termination: false };
    }

    // 2. Random Walk Controller
    function runRandomWalk(sensors, dt, algoState) {
        if (!algoState.mode) {
            algoState.mode = 'drive';
            algoState.timer = 0;
        }
        
        // Transition to stop-and-wait if target is under robot
        if (sensors.target_under_robot && algoState.mode !== 'stop_and_wait') {
            algoState.mode = 'stop_and_wait';
            algoState.timer = 0;
            return {
                command: 'stop',
                trigger_termination: false
            };
        }
        
        if (algoState.mode === 'stop_and_wait') {
            algoState.timer += dt;
            if (algoState.timer >= 0.5) {
                return {
                    command: 'stop',
                    trigger_termination: true
                };
            }
            return {
                command: 'stop',
                trigger_termination: false
            };
        }

        algoState.timer += dt;
        
        if (algoState.mode === 'drive') {
            // Drive forward for some random duration (2 to 4 seconds) unless timer hits limit
            if (algoState.timer > (2.0 + Math.random() * 2.0)) {
                algoState.mode = 'turn';
                algoState.timer = 0;
                algoState.turnDirection = Math.random() < 0.5 ? 'turn_left' : 'turn_right';
                algoState.turnDuration = 0.5 + Math.random() * 1.5; // Turn for 0.5 to 2.0 seconds
            }
            return {
                command: 'move_forward',
                trigger_termination: false
            };
        } else if (algoState.mode === 'turn') {
            // Turn in place
            if (algoState.timer > algoState.turnDuration) {
                algoState.mode = 'drive';
                algoState.timer = 0;
            }
            return {
                command: algoState.turnDirection,
                trigger_termination: false
            };
        }
        
        return { command: 'stop', trigger_termination: false };
    }

    // 3. Rotate-Scan then Drive Controller
    function runRotateScanDrive(sensors, dt, algoState, currentHeading) {
        if (!algoState.mode) {
            algoState.mode = 'init_scan';
            algoState.timer = 0;
            algoState.bestHeading = null;
            algoState.bestDistance = Infinity;
            algoState.scanHeadingStart = currentHeading;
        }
        
        // Transition to stop-and-wait if target is under robot
        if (sensors.target_under_robot && algoState.mode !== 'stop_and_wait') {
            algoState.mode = 'stop_and_wait';
            algoState.timer = 0;
            return {
                command: 'stop',
                trigger_termination: false
            };
        }
        
        if (algoState.mode === 'stop_and_wait') {
            algoState.timer += dt;
            if (algoState.timer >= 0.5) {
                return {
                    command: 'stop',
                    trigger_termination: true
                };
            }
            return {
                command: 'stop',
                trigger_termination: false
            };
        }
        
        algoState.timer += dt;
        
        if (algoState.mode === 'init_scan') {
            // Rotate in place to scan 360 degrees
            // Turn speed is 45 deg/s, scanning for 9.0 seconds ensures full 360 turn
            if (algoState.timer > 9.0) {
                if (algoState.bestHeading !== null) {
                    algoState.mode = 'align';
                    algoState.timer = 0;
                } else {
                    algoState.mode = 'search_advance';
                    algoState.timer = 0;
                }
                return { command: 'stop', trigger_termination: false };
            }
            
            if (sensors.target_visible) {
                if (sensors.target_distance_cm < algoState.bestDistance) {
                    algoState.bestDistance = sensors.target_distance_cm;
                    const absoluteTargetHeading = normalizeAngle(currentHeading + sensors.target_angle_deg);
                    algoState.bestHeading = absoluteTargetHeading;
                }
            }
            
            return {
                command: 'turn_right',
                trigger_termination: false
            };
            
        } else if (algoState.mode === 'search_advance') {
            if (algoState.timer > 2.0) {
                algoState.mode = 'init_scan';
                algoState.timer = 0;
                algoState.bestHeading = null;
                algoState.bestDistance = Infinity;
                algoState.scanHeadingStart = currentHeading;
                return { command: 'stop', trigger_termination: false };
            }
            
            return {
                command: 'move_forward',
                trigger_termination: false
            };
            
        } else if (algoState.mode === 'align') {
            const headingErr = normalizeAngle(algoState.bestHeading - currentHeading);
            
            if (Math.abs(headingErr) < 5.0) {
                algoState.mode = 'drive_to_target';
                algoState.timer = 0;
                return { command: 'stop', trigger_termination: false };
            }
            
            return {
                command: headingErr > 0 ? 'turn_left' : 'turn_right',
                trigger_termination: false
            };
            
        } else if (algoState.mode === 'drive_to_target') {
            if (sensors.target_visible) {
                if (sensors.target_angle_deg > 5.0) {
                    return { command: 'turn_left', trigger_termination: false };
                } else if (sensors.target_angle_deg < -5.0) {
                    return { command: 'turn_right', trigger_termination: false };
                }
            } else {
                if (algoState.timer > 1.5) {
                    algoState.mode = 'init_scan';
                    algoState.timer = 0;
                    algoState.bestHeading = null;
                    algoState.bestDistance = Infinity;
                    algoState.scanHeadingStart = currentHeading;
                    return { command: 'stop', trigger_termination: false };
                }
            }
            
            if (sensors.target_visible) {
                algoState.timer = 0;
            }
            
            return {
                command: 'move_forward',
                trigger_termination: false
            };
        }
        
        return { command: 'stop', trigger_termination: false };
    }

    // Main entry point for choosing algorithms
    function getControllerAction(algoName, sensors, dt, algoState, currentHeading) {
        switch (algoName) {
            case 'straight':
                return runStraightWalk(sensors, dt, algoState);
            case 'random':
                return runRandomWalk(sensors, dt, algoState);
            case 'rotate_scan':
                return runRotateScanDrive(sensors, dt, algoState, currentHeading);
            default:
                return { command: 'stop', trigger_termination: true };
        }
    }

    // Expose APIs
    return {
        Engine,
        getControllerAction,
        normalizeAngle,
        makeCCW,
        getPolygonArea,
        clipPolygon,
        isPointInConvexPolygon
    };
})();

if (typeof module !== 'undefined' && module.exports) {
    module.exports = Simulator;
} else if (typeof exports !== 'undefined') {
    exports.Simulator = Simulator;
} else {
    self.Simulator = Simulator;
}


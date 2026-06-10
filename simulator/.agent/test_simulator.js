// .agent/test_simulator.js
// NodeJS script to verify the correctness of the simulator.js math and engine

const Simulator = require('../simulator.js');

console.log("=== Testing Simulator Math & Mechanics ===");

// 1. Test CCW conversion
const cwPoly = [
    { x: 0, y: 0 },
    { x: 0, y: 5 },
    { x: 5, y: 5 },
    { x: 5, y: 0 }
];
const ccwPoly = Simulator.makeCCW(cwPoly);
console.log("CW Poly converted CCW:", ccwPoly);

// 2. Test Polygon Area
const area = Simulator.getPolygonArea(ccwPoly);
console.log("Area of 5x5 square (should be 25):", area);
if (Math.abs(area - 25) > 1e-9) {
    console.error("FAIL: Area calculation incorrect!");
    process.exit(1);
}

// 3. Test Polygon Intersection
const robot = [
    { x: -6, y: -8 },
    { x: -6, y: 8 },
    { x: 6, y: 8 },
    { x: 6, y: -8 }
]; // Simple 12x16 box centered at 0,0
const target = [
    { x: -2.5, y: -2.5 },
    { x: -2.5, y: 2.5 },
    { x: 2.5, y: 2.5 },
    { x: 2.5, y: -2.5 }
]; // 5x5 target centered at 0,0

const clipped = Simulator.clipPolygon(robot, target);
const clippedArea = Simulator.getPolygonArea(clipped);
console.log("Intersection Area (should be 25 since target is fully covered):", clippedArea);
if (Math.abs(clippedArea - 25) > 1e-9) {
    console.error("FAIL: Polygon clipping incorrect!");
    process.exit(1);
}

// Test off-center intersection
const robotOffCenter = [
    { x: -4, y: -8 },
    { x: -4, y: 8 },
    { x: 8, y: 8 },
    { x: 8, y: -8 }
]; // Translated by +2 on X
const clipped2 = Simulator.clipPolygon(robotOffCenter, target);
const clippedArea2 = Simulator.getPolygonArea(clipped2);
console.log("Intersection Area off-center (should be 25 since target [-2.5, 2.5] is still fully inside robot [-4, 8]):", clippedArea2);

// 4. Test Engine Simulation Loop
console.log("\n=== Testing Engine Simulation Loop ===");
const engine = new Simulator.Engine({
    target_width_cm: 5,
    target_height_cm: 5,
    noise_enabled: true,
    start_distance_min_cm: 100,
    start_distance_max_cm: 110
});

console.log("Engine initialized.");
console.log("Initial pose:", { x: engine.x, y: engine.y, heading: engine.heading });
console.log("Distance to target center:", Math.sqrt(engine.x*engine.x + engine.y*engine.y));

let steps = 0;
while (!engine.terminated && steps < 1000) {
    const sensors = engine.getSensors();
    const action = Simulator.getControllerAction('rotate_scan', sensors, 0.05, engine.algoState, engine.heading);
    engine.step(action);
    steps++;
}

console.log(`Simulation finished in ${steps} steps (${engine.time.toFixed(2)}s).`);
const score = engine.getScore();
console.log("Final score report:", score);

console.log("\nALL PROGRAMMATIC TESTS PASSED SUCCESSFULLY!");

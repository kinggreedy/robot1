// worker.js
// Web Worker for running batch simulations in the background

importScripts('simulator.js');

self.onmessage = function(e) {
    const { algoName, trialsCount, config } = e.data;
    
    // Aggregate metrics
    let fullCoverageCount = 0;
    let partialCoverageCount = 0;
    let touchedButFailedCount = 0;
    let movingTerminationFailures = 0;
    let totalScore = 0;
    let totalTime = 0;
    let failedCount = 0;
    
    const dt = config.time_step_s || 0.05;
    const progressInterval = Math.max(1, Math.floor(trialsCount / 100)); // Update every 1%
    
    for (let i = 0; i < trialsCount; i++) {
        const engine = new Simulator.Engine(config);
        
        // Run trial loop
        while (!engine.terminated) {
            const sensors = engine.getSensors();
            const action = Simulator.getControllerAction(
                algoName,
                sensors,
                dt,
                engine.algoState,
                engine.heading
            );
            engine.step(action);
        }
        
        // Record final score & metrics
        const result = engine.getScore();
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
            case 'moving_termination':
                // already counted in movingTerminationFailures, but let's categorize for base rate
                break;
            case 'failed':
            default:
                failedCount++;
                break;
        }
        
        // Send progress updates
        if ((i + 1) % progressInterval === 0 || i === trialsCount - 1) {
            const completed = i + 1;
            const currentAverageScore = totalScore / completed;
            const currentAverageTime = totalTime / completed;
            
            self.postMessage({
                type: 'progress',
                trialsCompleted: completed,
                progressPercent: Math.round((completed / trialsCount) * 100),
                stats: {
                    success_rate_full_coverage: (fullCoverageCount / completed) * 100,
                    partial_coverage_rate: (partialCoverageCount / completed) * 100,
                    touched_but_failed_rate: (touchedButFailedCount / completed) * 100,
                    moving_termination_failures: (movingTerminationFailures / completed) * 100,
                    average_score: currentAverageScore,
                    average_time_s: currentAverageTime
                }
            });
        }
    }
    
    // Send final results
    self.postMessage({
        type: 'complete',
        totalTrials: trialsCount,
        stats: {
            success_rate_full_coverage: (fullCoverageCount / trialsCount) * 100,
            partial_coverage_rate: (partialCoverageCount / trialsCount) * 100,
            touched_but_failed_rate: (touchedButFailedCount / trialsCount) * 100,
            moving_termination_failures: (movingTerminationFailures / trialsCount) * 100,
            average_score: totalScore / trialsCount,
            average_time_s: totalTime / trialsCount
        }
    });
};

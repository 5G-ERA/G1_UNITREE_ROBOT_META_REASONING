
import random, math
from escmr_reasoner import *

def build_reasoner():
    meta=("safety","progression")
    efficient=AnalogyQoESpecification("efficient_navigation",meta,{"safety":FourSemanticRegionCalibration(1.0,0.8,0.5),"progression":FourSemanticRegionCalibration(1.0,0.6,0.1)},{"safety":0.4,"progression":0.6},{"safety":1.0,"progression":0.2})
    conservative=AnalogyQoESpecification("conservative_navigation",meta,{"safety":FourSemanticRegionCalibration(0.8,0.6,0.5),"progression":FourSemanticRegionCalibration(1.0,0.4,0.05)},{"safety":0.7,"progression":0.3},{"safety":1.0,"progression":0.2})
    cal=ReasonerCalibration((efficient,conservative),{"safety":SensorCalibration(0.95,0.12),"progression":SensorCalibration(1.0,0.08)},{"efficient_navigation":0.6,"conservative_navigation":0.4},runtime_config=ReasonerRuntimeConfig(0.5,2.0))
    return ExperienceScopedCapabilityMetaReasoner.from_calibration(cal)

def check(cond,name,passed):
    assert cond,name; passed.append(name)

def run_robustness_tests():
    passed=[]
    # Initial incomplete data cases
    r=build_reasoner(); fb=r.evaluate_latest_reading({})
    check(fb.terminate, "empty initial reading terminates/fallbacks without crash", passed)
    check(len(r.experience_memory)==1, "empty initial reading is remembered as a bounded snapshot", passed)
    r=build_reasoner(); fb=r.evaluate_latest_reading({"progression":1.2})
    check(fb.terminate, "missing safety is dangerous/insufficient", passed)
    r=build_reasoner(); fb=r.evaluate_latest_reading({"safety":1.2})
    check(fb.terminate, "missing progression is dangerous/insufficient", passed)
    r=build_reasoner(); fb=r.evaluate_latest_reading({"safety":float('nan'),"progression":1.2})
    check(fb.terminate, "NaN safety is dangerous/insufficient", passed)
    r=build_reasoner(); fb=r.evaluate_latest_reading({"safety":float('inf'),"progression":1.2})
    check(fb.terminate, "infinite safety is dangerous/insufficient", passed)
    # Recovery after incomplete startup
    r=build_reasoner(); r.evaluate_latest_reading({}); fb=r.evaluate_latest_reading({"safety":1.2,"progression":1.2})
    check(not fb.terminate and fb.active_after=="efficient_navigation", "complete reading after incomplete startup recovers to efficient", passed)
    # Noise uncertainty near boundary
    r=build_reasoner(); r.evaluate_latest_reading({"safety":1.2,"progression":1.2})
    fb=r.evaluate_latest_reading({"safety":1.02,"progression":1.1},{"safety":0.1,"progression":1.0})
    tr=fb.analogy_evaluations["efficient_navigation"].dimension_traces["safety"]
    check(tr.uncertainty_margin>0.1, "low reliability widens uncertainty margin", passed)
    check(tr.tension_direction==TensionDirection.TOWARD_HIGHER_CONCERN, "near-boundary noisy safety increases tension", passed)
    # Noisy open corridor should not cause conservative switch or termination
    random.seed(7)
    r=build_reasoner(); switches=0; terminations=0
    active=[]
    for _ in range(120):
        reading={"safety":random.gauss(1.2,0.03),"progression":random.gauss(1.2,0.03)}
        fb=r.evaluate_latest_reading(reading)
        active.append(fb.active_after)
        switches += int(fb.recommended_action==ReasonerFeedbackAction.SWITCH_ANALOGY)
        terminations += int(fb.terminate)
    check(terminations==0, "noisy open corridor has zero terminations", passed)
    check(set(active)=={"efficient_navigation"}, "noisy open corridor remains efficient", passed)
    check(switches==0, "noisy open corridor has no analogy chatter", passed)
    # Noisy doorway transition should switch at least once but not chatter excessively
    random.seed(11)
    r=build_reasoner(); sequence=[]; switch_count=0; terminations=0
    phases=[(1.2,1.2,20),(0.9,1.1,20),(0.72,0.88,20),(0.9,0.5,20),(1.2,1.2,20)]
    for s,p,n in phases:
        for _ in range(n):
            fb=r.evaluate_latest_reading({"safety":random.gauss(s,0.025),"progression":random.gauss(p,0.025)})
            sequence.append(fb.active_after)
            switch_count += int(fb.recommended_action==ReasonerFeedbackAction.SWITCH_ANALOGY)
            terminations += int(fb.terminate)
    check(terminations==0, "noisy doorway sequence has zero terminations", passed)
    check("conservative_navigation" in sequence, "noisy doorway sequence reaches conservative", passed)
    check(sequence[-1]=="efficient_navigation", "noisy doorway sequence returns efficient", passed)
    check(switch_count<=4, "noisy doorway sequence has limited switching/chatter", passed)
    # Memory limit after noisy run
    check(len(r.experience_memory)<=60, "noisy run respects 60-tick memory window", passed)
    return passed, {"open_switches":switches,"doorway_switches":switch_count,"doorway_final":sequence[-1],"memory_len":len(r.experience_memory)}

if __name__=="__main__":
    passed, summary=run_robustness_tests()
    print(f"ROBUSTNESS TESTS PASSED: {len(passed)}")
    for p in passed: print(" -",p)
    print("SUMMARY",summary)


from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from enum import Enum, IntEnum
from typing import Deque, Dict, Optional, Tuple
import math, time
EPS = 1e-12

class ExperienceRegion(IntEnum):
    EXPECTED = 0
    ADAPTIVE = 1
    HIGH_CONCERN = 2
    DANGEROUS_INSUFFICIENT = 3

class TensionDirection(IntEnum):
    TOWARD_LOWER_CONCERN = -1
    STABLE = 0
    TOWARD_HIGHER_CONCERN = 1

class ReasonerFeedbackAction(str, Enum):
    KEEP_CURRENT_ANALOGY = "KEEP_CURRENT_ANALOGY"
    SWITCH_ANALOGY = "SWITCH_ANALOGY"
    TERMINATE_OR_FALLBACK = "TERMINATE_OR_FALLBACK"
    KNOWN_INSUFFICIENCY = "KNOWN_INSUFFICIENCY"

REGION_NAME = {
    ExperienceRegion.EXPECTED: "expected",
    ExperienceRegion.ADAPTIVE: "adaptive",
    ExperienceRegion.HIGH_CONCERN: "high_concern",
    ExperienceRegion.DANGEROUS_INSUFFICIENT: "dangerous_insufficient",
}
DIRECTION_NAME = {
    TensionDirection.TOWARD_LOWER_CONCERN: "toward_lower_concern",
    TensionDirection.STABLE: "stable",
    TensionDirection.TOWARD_HIGHER_CONCERN: "toward_higher_concern",
}
CONCERN_BY_REGION_AND_DIRECTION = {
    ExperienceRegion.EXPECTED: {TensionDirection.TOWARD_LOWER_CONCERN:0.0,TensionDirection.STABLE:0.0,TensionDirection.TOWARD_HIGHER_CONCERN:0.2},
    ExperienceRegion.ADAPTIVE: {TensionDirection.TOWARD_LOWER_CONCERN:0.2,TensionDirection.STABLE:0.4,TensionDirection.TOWARD_HIGHER_CONCERN:0.55},
    ExperienceRegion.HIGH_CONCERN: {TensionDirection.TOWARD_LOWER_CONCERN:0.55,TensionDirection.STABLE:0.7,TensionDirection.TOWARD_HIGHER_CONCERN:0.85},
    ExperienceRegion.DANGEROUS_INSUFFICIENT: {TensionDirection.TOWARD_LOWER_CONCERN:0.85,TensionDirection.STABLE:1.0,TensionDirection.TOWARD_HIGHER_CONCERN:1.0},
}

@dataclass(frozen=True)
class FourSemanticRegionCalibration:
    expected_boundary: float
    adaptive_boundary: float
    dangerous_boundary: float
    def classify_experience_region(self, value: float) -> ExperienceRegion:
        if not math.isfinite(value): return ExperienceRegion.DANGEROUS_INSUFFICIENT
        if value > self.expected_boundary: return ExperienceRegion.EXPECTED
        if value > self.adaptive_boundary: return ExperienceRegion.ADAPTIVE
        if value > self.dangerous_boundary: return ExperienceRegion.HIGH_CONCERN
        return ExperienceRegion.DANGEROUS_INSUFFICIENT

@dataclass(frozen=True)
class SensorCalibration:
    default_reliability: float = 1.0
    reliability_sensitivity: float = 0.0

@dataclass(frozen=True)
class ReasonerRuntimeConfig:
    frequency_hz: float
    memory_duration_minutes: float
    hysteresis_margin: float = 0.05
    forecast_gain: float = 1.0
    strong_task_pressure_ratio: float = 1.5
    def __post_init__(self):
        if self.frequency_hz <= 0: raise ValueError("frequency_hz must be positive")
        if self.memory_duration_minutes <= 0: raise ValueError("memory_duration_minutes must be positive")
    @property
    def memory_window_ticks(self) -> int:
        return max(1, int(math.ceil(self.frequency_hz * 60.0 * self.memory_duration_minutes)))

@dataclass(frozen=True)
class AnalogyQoESpecification:
    name: str
    meta_parameters: Tuple[str, ...]
    region_calibration: Dict[str, FourSemanticRegionCalibration]
    attention_vector: Dict[str, float]
    lethality_vector: Dict[str, float]
    hard_alert_regions: Tuple[ExperienceRegion, ...] = (ExperienceRegion.DANGEROUS_INSUFFICIENT,)
    def __post_init__(self):
        dims=set(self.meta_parameters)
        if dims != set(self.region_calibration) or dims != set(self.attention_vector) or dims != set(self.lethality_vector):
            raise ValueError(f"{self.name}: QoE dictionaries must match meta_parameters")
        total=sum(self.attention_vector.values())
        if total <= 0: raise ValueError(f"{self.name}: attention_vector must sum positive")
        object.__setattr__(self,"attention_vector",{k:v/total for k,v in self.attention_vector.items()})
        if not all(0 <= v <= 1 for v in self.lethality_vector.values()):
            raise ValueError(f"{self.name}: lethality must be in [0,1]")

@dataclass(frozen=True)
class ReasonerCalibration:
    analogy_library: Tuple[AnalogyQoESpecification, ...]
    sensor_calibration: Dict[str, SensorCalibration]
    initial_analogy_weights: Dict[str, float]
    initial_active_analogy: Optional[str] = None
    runtime_config: ReasonerRuntimeConfig = ReasonerRuntimeConfig(0.5,2.0)

@dataclass
class DimensionTrace:
    value: float; baseline_value: float; forecast_value: float; reliability: float; uncertainty_margin: float
    belief_bound: float; plausibility_bound: float; region: ExperienceRegion; belief_region: ExperienceRegion
    plausibility_region: ExperienceRegion; tension_direction: TensionDirection; concern: float

@dataclass
class AnalogyEvaluationTrace:
    analogy_name: str; dimension_traces: Dict[str, DimensionTrace]; local_deployability_tension: float
    comfort_score: float; local_plausibility: float; local_belief: float; belief_plausibility_gap: float
    survivability_gate: bool; alert_active: bool; locally_stable: bool; tension_slope: float

@dataclass
class ReasonerFeedback:
    tick_index: int; timestamp: float; active_before: str; active_after: str; recommended_action: ReasonerFeedbackAction
    keep_current: bool; switch_to: Optional[str]; terminate: bool; explanation: str
    analogy_evaluations: Dict[str, AnalogyEvaluationTrace]; task_level_contribution: Dict[str, float]
    task_weighted_comfort: Dict[str, float]; memory_window_ticks: int; memory_duration_minutes: float

class ExperienceScopedCapabilityMetaReasoner:
    def __init__(self, calibration: ReasonerCalibration):
        self.calibration=calibration
        self.analogy_library={a.name:a for a in calibration.analogy_library}
        if not self.analogy_library: raise ValueError("analogy_library cannot be empty")
        self.initial_analogy_weights=self._normalise_analogy_weights(calibration.initial_analogy_weights)
        self.active_analogy=calibration.initial_active_analogy or max(self.initial_analogy_weights,key=self.initial_analogy_weights.get)
        if self.active_analogy not in self.analogy_library: raise ValueError("initial_active_analogy must be known")
        self.runtime_config=calibration.runtime_config
        self.tick_index=0
        self.experience_memory: Deque[Dict[str,float]]=deque(maxlen=self.runtime_config.memory_window_ticks)
        self.tension_memory={name:deque(maxlen=self.runtime_config.memory_window_ticks) for name in self.analogy_library}
    @classmethod
    def from_calibration(cls, calibration): return cls(calibration)
    def _normalise_analogy_weights(self, weights):
        unknown=set(weights)-set(self.analogy_library)
        if unknown: raise ValueError(f"unknown analogies: {sorted(unknown)}")
        total=sum(max(0.0,weights.get(n,0.0)) for n in self.analogy_library)
        if total <= EPS: return {n:1/len(self.analogy_library) for n in self.analogy_library}
        return {n:max(0.0,weights.get(n,0.0))/total for n in self.analogy_library}
    def evaluate_latest_reading(self, latest_sensor_reading, sensor_reliability=None, task_attention=None, analogy_contribution=None, timestamp=None):
        now=time.time() if timestamp is None else timestamp
        perceived=dict(latest_sensor_reading)
        evals={n:self.generate_local_deployability_tension(spec,perceived,sensor_reliability) for n,spec in self.analogy_library.items()}
        if not any(e.survivability_gate for e in evals.values()):
            fb=self._feedback(now,self.active_analogy,self.active_analogy,ReasonerFeedbackAction.KNOWN_INSUFFICIENCY,False,None,True,"No analogy survived local survivability gating.",evals,{n:0.0 for n in evals},{n:0.0 for n in evals})
            self._remember(perceived,evals); self.tick_index+=1; return fb
        if analogy_contribution is not None: contrib=self._normalise_analogy_weights(analogy_contribution)
        elif task_attention is not None: contrib=self.estimate_task_level_analogy_contribution(task_attention)
        else: contrib=dict(self.initial_analogy_weights)
        candidate,twc=self.cme_select_candidate_analogy(evals,contrib)
        current=self.active_analogy
        if not any(e.survivability_gate and not e.alert_active for e in evals.values()):
            fb=self._feedback(now,current,current,ReasonerFeedbackAction.TERMINATE_OR_FALLBACK,False,None,True,"All candidate analogies were infeasible or vetoed by dangerous/insufficient regions.",evals,contrib,twc)
        else:
            preferred=max(self.initial_analogy_weights,key=self.initial_analogy_weights.get)
            ede_switch=candidate!=current and self.ede_verify_deployability(evals[current],evals[candidate],twc[current],twc[candidate])
            restore=candidate!=current and candidate==preferred and twc[candidate]>=twc[current]-EPS and evals[candidate].local_deployability_tension<=evals[current].local_deployability_tension+EPS and evals[candidate].survivability_gate and not evals[candidate].alert_active
            if ede_switch or restore:
                self.active_analogy=candidate
                fb=self._feedback(now,current,candidate,ReasonerFeedbackAction.SWITCH_ANALOGY,False,candidate,False,f"Switch from {current} to {candidate}.",evals,contrib,twc)
            else:
                fb=self._feedback(now,current,current,ReasonerFeedbackAction.KEEP_CURRENT_ANALOGY,True,None,False,f"Keep {current}.",evals,contrib,twc)
        self._remember(perceived,evals); self.tick_index+=1; return fb
    def _feedback(self, ts,before,after,action,keep,switch_to,terminate,explanation,evals,contrib,twc):
        return ReasonerFeedback(self.tick_index,ts,before,after,action,keep,switch_to,terminate,explanation,evals,contrib,twc,self.runtime_config.memory_window_ticks,self.runtime_config.memory_duration_minutes)
    def _memory_baseline_for(self, mp, cur):
        vals=[s[mp] for s in self.experience_memory if mp in s and math.isfinite(s[mp])]
        return cur if not vals else sum(vals)/len(vals)
    def _tension_baseline_for(self, name, cur):
        vals=list(self.tension_memory[name]); return cur if not vals else sum(vals)/len(vals)
    @staticmethod
    def _clip01(v):
        if not math.isfinite(v): return 0.0
        return max(0.0,min(1.0,v))
    def _sensor_reliability_for(self, mp, live):
        if live and mp in live: return self._clip01(float(live[mp]))
        if mp in self.calibration.sensor_calibration: return self._clip01(self.calibration.sensor_calibration[mp].default_reliability)
        return 1.0
    def _reliability_sensitivity_for(self, mp):
        return self.calibration.sensor_calibration.get(mp,SensorCalibration()).reliability_sensitivity
    def compute_plausibility_belief_envelope(self, cur, baseline, reliability, sensitivity):
        forecast=cur+self.runtime_config.forecast_gain*(cur-baseline) if math.isfinite(cur) and math.isfinite(baseline) else float('nan')
        margin=sensitivity*(1-self._clip01(reliability))
        if math.isfinite(cur) and math.isfinite(forecast): return forecast, min(cur,forecast)-margin, max(cur,forecast)+margin, margin
        return forecast,float('nan'),float('nan'),margin
    @staticmethod
    def infer_tension_direction_from_regions(region, belief_region, plausibility_region):
        if belief_region>region: return TensionDirection.TOWARD_HIGHER_CONCERN
        if plausibility_region<region: return TensionDirection.TOWARD_LOWER_CONCERN
        return TensionDirection.STABLE
    def generate_local_deployability_tension(self, analogy, perceived, reliability_dict):
        traces={}; tension=0.0; plaus=0.0; belief_mult=1.0; alert=False; survivable=True
        for mp in analogy.meta_parameters:
            value=float(perceived.get(mp,float('nan')))
            baseline=self._memory_baseline_for(mp,value)
            reliability=self._sensor_reliability_for(mp,reliability_dict)
            sens=self._reliability_sensitivity_for(mp)
            forecast,bel,pl,margin=self.compute_plausibility_belief_envelope(value,baseline,reliability,sens)
            calib=analogy.region_calibration[mp]
            region=calib.classify_experience_region(value); bel_region=calib.classify_experience_region(bel); pl_region=calib.classify_experience_region(pl)
            direction=self.infer_tension_direction_from_regions(region,bel_region,pl_region)
            concern=CONCERN_BY_REGION_AND_DIRECTION[region][direction]
            attention=analogy.attention_vector[mp]; tension+=attention*concern
            clipped=self._clip01(value); plaus+=attention*clipped
            criticality=1-clipped; lethality=analogy.lethality_vector[mp]
            belief_mult*=max(0.0,1.0-max(attention,lethality)*criticality)
            if region in analogy.hard_alert_regions: alert=True
            if lethality>=1.0 and criticality>=1.0-EPS: survivable=False
            traces[mp]=DimensionTrace(value,baseline,forecast,reliability,margin,bel,pl,region,bel_region,pl_region,direction,concern)
        belief=plaus*belief_mult; slope=tension-self._tension_baseline_for(analogy.name,tension)
        return AnalogyEvaluationTrace(analogy.name,traces,tension,max(0,1-tension),plaus,belief,max(0,plaus-belief),survivable,alert,slope<=EPS,slope)
    def estimate_task_level_analogy_contribution(self, task_attention):
        raw={}
        for name,analogy in self.analogy_library.items():
            vals=[]
            for mp,tw in task_attention.items():
                aw=analogy.attention_vector.get(mp,0.0); vals.append(1.0 if aw<=tw+EPS else tw)
            raw[name]=min(vals) if vals else 0.0
        return self._normalise_analogy_weights(raw)
    def cme_select_candidate_analogy(self, evals, contrib):
        twc={n:contrib[n]*evals[n].comfort_score for n in evals}
        feasible={n:e for n,e in evals.items() if e.survivability_gate and not e.alert_active}
        if not feasible: return self.active_analogy,twc
        return max(feasible,key=lambda n:twc[n]),twc
    def ede_verify_deployability(self,current_eval,candidate_eval,current_twc,candidate_twc):
        improvement=candidate_twc>current_twc+self.runtime_config.hysteresis_margin
        worsening=any(t.tension_direction==TensionDirection.TOWARD_HIGHER_CONCERN for t in current_eval.dimension_traces.values())
        unstable=(not current_eval.locally_stable) or worsening
        pressure=candidate_twc>self.runtime_config.strong_task_pressure_ratio*max(current_twc,EPS)
        return bool(candidate_eval.survivability_gate and not candidate_eval.alert_active and improvement and (unstable or pressure))
    def _remember(self, perceived, evals):
        self.experience_memory.append(dict(perceived))
        for n,e in evals.items(): self.tension_memory[n].append(e.local_deployability_tension)

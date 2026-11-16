"""Diagnostic relative bridge: failed real-sequence cycle consistency; NOT for live.

Constant source bias tests pass, but the recorded hand sequence accumulates about
119 degrees of same-source anchor disagreement. Do not deploy or smooth this away.
"""
import numpy as np
from .palm_reference import proper_rotation


class OrientationBridge:
    def __init__(self, stale_after_s):
        self.stale=float(stale_after_s)
        self.source=None
        self.anchor=self.output=None
        self.last_sources={}
        self.last_output_time=None
        self.last_input_time=-np.inf

    def update(self, sources, timestamp, preferred='depth_flow'):
        if not np.isfinite(timestamp) or timestamp<=self.last_input_time:
            return dict(valid=False,reason='nonmonotonic_time',rotation=None)
        self.last_input_time=timestamp
        sources={key:np.asarray(value,float) for key,value in sources.items() if proper_rotation(value)}
        if not sources:
            return dict(valid=False,reason='no_current_orientation',rotation=None)
        selected=preferred if preferred in sources else next(iter(sources))
        switched=self.source is not None and self.source!=selected
        if self.source is None:
            self.anchor=sources[selected].T
        elif switched:
            if selected not in self.last_sources or timestamp-self.last_output_time>self.stale:
                # Continue the active source if still observable. Otherwise no bridge.
                if self.source in sources:
                    selected=self.source
                    switched=False
                else:
                    return dict(valid=False,reason='source_switch_without_recent_overlap',rotation=None)
            else:
                self.anchor=self.last_sources[selected].T@self.output
        self.output=sources[selected]@self.anchor
        self.source=selected
        self.last_sources={k:v.copy() for k,v in sources.items()}
        self.last_output_time=timestamp
        return dict(valid=True,reason='relative_mapping_not_anatomical_pose',rotation=self.output.tolist(),
                    source=selected,switched=switched)

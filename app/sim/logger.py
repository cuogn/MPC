from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List
import pandas as pd

@dataclass
class SimLogger:
    records: Dict[str, List[float]] = field(default_factory=dict)

    def append(self, **kwargs: float) -> None:
        for k, v in kwargs.items():
            self.records.setdefault(k, []).append(float(v))

    def to_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame(self.records)

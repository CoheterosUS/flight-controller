"""Scenario pass/fail result and a small assertion helper."""


class ScenarioResult:
    def __init__(self, scenario_id):
        self.scenario_id = scenario_id
        self.checks = []  # (ok, message)
        self.blocked = None  # reason string when the scenario needs unlanded firmware

    def check(self, ok, message):
        self.checks.append((bool(ok), message))
        return ok

    def block(self, reason):
        self.blocked = reason

    @property
    def passed(self):
        return not self.blocked and all(ok for ok, _ in self.checks) and bool(self.checks)

    def report(self):
        if self.blocked:
            return f"{self.scenario_id}: BLOCKED — {self.blocked}"
        lines = [f"{self.scenario_id}: {'PASS' if self.passed else 'FAIL'}"]
        for ok, msg in self.checks:
            lines.append(f"    [{'ok' if ok else 'XX'}] {msg}")
        return "\n".join(lines)

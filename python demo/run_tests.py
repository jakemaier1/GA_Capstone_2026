"""Run the communications tests with descriptions for the team."""

from pathlib import Path
import sys
import unittest


class TeamTestResult(unittest.TextTestResult):
    def getDescription(self, test):
        # Preserve unittest's case details when a subtest fails.
        if getattr(test, "params", None):
            return super().getDescription(test)
        return test.shortDescription() or str(test)


def main():
    print("Airport rover — secure communications tests")
    print("ok = PASS | FAIL = unexpected result | ERROR = unexpected exception")
    print("Rejection checks PASS when the unsafe connection or message is rejected.\n")
    folder = Path(__file__).resolve().parent
    suite = unittest.defaultTestLoader.discover(str(folder / "tests"))
    result = unittest.TextTestRunner(
        stream=sys.stdout, verbosity=2, resultclass=TeamTestResult,
    ).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())

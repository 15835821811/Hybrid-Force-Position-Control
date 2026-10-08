"""python -m v6_mujoco.postgrasp_campaign: gated campaign CLI."""
import argparse
import json
from .contract import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--stage", choices=("all", "S", "P", "G", "L", "V"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--report", action="store_true")
    args = parser.parse_args()
    if args.resume and not (args.stage or args.prepare or args.validate_only or args.report):
        args.stage = "all"
    if args.prepare:
        result = prepare()
        print(json.dumps({"prepared": True, "fine_cases_frozen": True, "manifest": result["schema_version"]}))
    if args.stage == "S":
        from .restricted import run
        print(json.dumps({"restricted_fixture_domain": run(args.resume)["status"]}))
    elif args.stage:
        from .campaign import execute
        execute(args.stage, args.resume)
    if args.validate_only:
        from .validation import validate_all
        validate_all()
    if args.report:
        from .report import build
        build()


if __name__ == "__main__":
    main()

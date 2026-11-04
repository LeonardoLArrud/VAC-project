"""params.ini sets the constants each module declares as None: load("<section>", globals()) after them.
Values are Python literals; key_deg is in degrees and stored in radians. Unknown or missing key: error."""
import ast
import configparser
import math
import os

PATH = os.environ.get("ROBO_PARAMS") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "params.ini")


def load(section, g):
    ini = configparser.ConfigParser(inline_comment_prefixes=("#",), interpolation=None)
    ini.optionxform = str
    if not ini.read(PATH):
        raise FileNotFoundError(f"parameters file not found: {PATH}")
    done = set()
    for key, text in (ini.items(section) if ini.has_section(section) else []):
        name = key.removesuffix("_deg")
        if name not in g:
            raise KeyError(f"{PATH} [{section}]: unknown parameter {key}")
        value = ast.literal_eval(text)
        g[name] = math.radians(value) if key.endswith("_deg") else value
        done.add(name)
    missing = [k for k, v in g.items() if k.isupper() and v is None and k not in done]
    if missing:
        raise KeyError(f"{PATH} [{section}]: missing {', '.join(missing)}")

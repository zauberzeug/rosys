import os
import asyncio
from runtime import Runtime
from world.mode import Mode


has_esp = os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)


loop = asyncio.get_event_loop()

print("starting runtime")
loop.run_until_complete(runtime.run(10))
print("finshed runtime")

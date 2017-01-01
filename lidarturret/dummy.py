
live = True

try:
    if live:
        live = True
except NameError:
    live = False

if live:
    from dummy.dummy_runner import run_dummy
    run_dummy()
else:
    from dummy.dummy_simulator import simulate_dummy
    simulate_dummy()

# live = True

try:
    live = True

    from dummy.dummy_runner import run_dummy

    run_dummy()
except NameError:
    from dummy.dummy_simulator import simulate_dummy

    simulate_dummy()
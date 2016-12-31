
# live = True

try:
    print("Running live:", live)

    from dummy.dummy_runner import run_dummy

    run_dummy()
except NameError:
    from dummy.dummy_simulator import simulate_dummy

    simulate_dummy()
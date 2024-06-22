import threading
import subprocess
import os

def popen_and_call(on_exit, report_id, popen_args):
    """
    Runs the given args in a subprocess.Popen, and then calls the function
    on_exit when the subprocess completes.
    on_exit is a callable object, and popen_args is a list/tuple of args that
    would give to subprocess.Popen.
    """
    def run_in_thread(on_exit, report_id, popen_args):
        print(popen_args, flush=True)
        proc = subprocess.Popen(popen_args)
        proc.wait()
        on_exit(report_id)
        return
    thread = threading.Thread(target=run_in_thread, args=(on_exit, report_id, popen_args))
    thread.start()
    # returns immediately after the thread starts
    return thread
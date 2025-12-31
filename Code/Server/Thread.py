# -*- coding: utf-8 -*-
"""
Safe thread management for Freenove Hexapod Robot Server.
Provides Event-based graceful shutdown instead of dangerous forced termination.
"""

import threading
import time
from typing import Optional, Callable, Any, List
from logger import get_logger

logger = get_logger()


class ManagedThread:
    """
    Wrapper for threading.Thread with graceful shutdown support.
    Uses threading.Event for safe termination.
    """

    def __init__(
        self,
        target: Callable,
        name: Optional[str] = None,
        args: tuple = (),
        kwargs: Optional[dict] = None,
        daemon: bool = False
    ):
        """
        Initialize managed thread.

        Args:
            target: Function to run in thread
            name: Thread name for identification
            args: Positional arguments for target function
            kwargs: Keyword arguments for target function
            daemon: Whether thread is daemon
        """
        self.shutdown_event = threading.Event()
        self._target = target
        self._args = args
        self._kwargs = kwargs or {}
        self._thread: Optional[threading.Thread] = None
        self._name = name or f"ManagedThread-{id(self)}"
        self._daemon = daemon
        self._exception: Optional[Exception] = None

    def _wrapped_target(self):
        """Wrapper that provides shutdown event to target function."""
        try:
            # If target accepts shutdown_event parameter, pass it
            import inspect
            sig = inspect.signature(self._target)
            if 'shutdown_event' in sig.parameters:
                self._target(*self._args, shutdown_event=self.shutdown_event, **self._kwargs)
            else:
                # Otherwise, target must check self.shutdown_event manually
                self._target(*self._args, **self._kwargs)
        except Exception as e:
            self._exception = e
            logger.exception(f"Exception in thread {self._name}")

    def start(self) -> 'ManagedThread':
        """
        Start the thread.

        Returns:
            Self for method chaining
        """
        if self._thread is not None and self._thread.is_alive():
            logger.warning(f"Thread {self._name} already running")
            return self

        self._thread = threading.Thread(
            target=self._wrapped_target,
            name=self._name,
            daemon=self._daemon
        )
        self._thread.start()
        logger.info(f"Started thread {self._name}")
        return self

    def stop(self, timeout: float = 5.0) -> bool:
        """
        Signal thread to stop and wait for completion.

        Args:
            timeout: Max seconds to wait for thread to stop

        Returns:
            True if thread stopped within timeout, False otherwise
        """
        if self._thread is None:
            return True

        if not self._thread.is_alive():
            logger.debug(f"Thread {self._name} already stopped")
            return True

        logger.info(f"Stopping thread {self._name}")
        self.shutdown_event.set()

        self._thread.join(timeout=timeout)

        if self._thread.is_alive():
            logger.error(f"Thread {self._name} did not stop within {timeout}s timeout")
            return False

        logger.info(f"Thread {self._name} stopped successfully")
        return True

    def is_alive(self) -> bool:
        """Check if thread is running."""
        return self._thread is not None and self._thread.is_alive()

    def should_stop(self) -> bool:
        """Check if shutdown has been requested."""
        return self.shutdown_event.is_set()

    def get_exception(self) -> Optional[Exception]:
        """Get exception raised in thread, if any."""
        return self._exception

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup."""
        self.stop()
        return False


class ThreadController:
    """
    Manages multiple threads with coordinated shutdown.
    Provides centralized control over thread lifecycle.
    """

    def __init__(self):
        """Initialize thread controller."""
        self.threads: List[ManagedThread] = []
        self._lock = threading.Lock()

    def create_thread(
        self,
        target: Callable,
        name: Optional[str] = None,
        args: tuple = (),
        kwargs: Optional[dict] = None,
        daemon: bool = False,
        auto_start: bool = False
    ) -> ManagedThread:
        """
        Create a new managed thread.

        Args:
            target: Function to run in thread
            name: Thread name
            args: Positional arguments
            kwargs: Keyword arguments
            daemon: Daemon flag
            auto_start: Start immediately after creation

        Returns:
            ManagedThread instance
        """
        thread = ManagedThread(
            target=target,
            name=name,
            args=args,
            kwargs=kwargs,
            daemon=daemon
        )

        with self._lock:
            self.threads.append(thread)

        if auto_start:
            thread.start()

        return thread

    def stop_all(self, timeout: float = 5.0) -> bool:
        """
        Stop all managed threads.

        Args:
            timeout: Max seconds to wait per thread

        Returns:
            True if all threads stopped successfully
        """
        logger.info("Stopping all threads")
        all_stopped = True

        with self._lock:
            threads_to_stop = list(self.threads)

        for thread in threads_to_stop:
            if not thread.stop(timeout=timeout):
                all_stopped = False

        if all_stopped:
            logger.info("All threads stopped successfully")
        else:
            logger.warning("Some threads did not stop within timeout")

        return all_stopped

    def get_alive_threads(self) -> List[ManagedThread]:
        """Get list of currently running threads."""
        with self._lock:
            return [t for t in self.threads if t.is_alive()]

    def wait_all(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for all threads to complete.

        Args:
            timeout: Max seconds to wait (None = wait forever)

        Returns:
            True if all threads completed, False if timeout
        """
        start_time = time.time()

        with self._lock:
            threads_to_wait = list(self.threads)

        for thread in threads_to_wait:
            if timeout is not None:
                elapsed = time.time() - start_time
                remaining = max(0, timeout - elapsed)
                if remaining == 0:
                    return False
            else:
                remaining = None

            if thread._thread is not None:
                thread._thread.join(timeout=remaining)

        return all(not t.is_alive() for t in threads_to_wait)

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - stop all threads."""
        self.stop_all()
        return False


# Backward compatibility functions for existing code
def stop_thread(thread: threading.Thread, timeout: float = 5.0) -> bool:
    """
    DEPRECATED: Backward compatibility wrapper.
    This is a safe fallback - cannot force stop standard threads.

    Args:
        thread: Thread to stop (must be ManagedThread or have shutdown_event)
        timeout: Timeout in seconds

    Returns:
        True if stopped, False if unable to stop safely
    """
    logger.warning(
        "stop_thread() is deprecated and unsafe. "
        "Use ManagedThread or ThreadController instead."
    )

    # If it's a ManagedThread, use proper stop
    if isinstance(thread, ManagedThread):
        return thread.stop(timeout=timeout)

    # Check if thread has shutdown_event attribute
    if hasattr(thread, 'shutdown_event'):
        thread.shutdown_event.set()
        thread.join(timeout=timeout)
        return not thread.is_alive()

    # Cannot safely stop a standard thread
    logger.error(
        f"Cannot safely stop thread {thread.name}. "
        "Thread must be ManagedThread or have shutdown_event attribute."
    )
    return False


def create_stoppable_thread(
    target: Callable,
    name: Optional[str] = None,
    args: tuple = (),
    daemon: bool = False
) -> ManagedThread:
    """
    Helper function to create a thread with shutdown support.

    Args:
        target: Function to run
        name: Thread name
        args: Arguments to target
        daemon: Daemon flag

    Returns:
        ManagedThread instance
    """
    return ManagedThread(target=target, name=name, args=args, daemon=daemon)


if __name__ == "__main__":
    # Example usage
    from logger import setup_logger
    setup_logger(log_level='DEBUG')

    def worker_function(shutdown_event: threading.Event):
        """Example worker that checks shutdown event."""
        counter = 0
        while not shutdown_event.is_set():
            print(f"Working... {counter}")
            counter += 1
            time.sleep(0.5)
        print("Worker stopped gracefully")

    # Example 1: Single managed thread
    print("\n=== Example 1: Single Managed Thread ===")
    thread = ManagedThread(target=worker_function, name="Worker-1")
    thread.start()
    time.sleep(2)
    thread.stop(timeout=3.0)

    # Example 2: Using context manager
    print("\n=== Example 2: Context Manager ===")
    with ManagedThread(target=worker_function, name="Worker-2") as thread:
        time.sleep(2)
    # Automatically stopped on context exit

    # Example 3: Thread controller
    print("\n=== Example 3: Thread Controller ===")
    with ThreadController() as controller:
        controller.create_thread(worker_function, name="Worker-3", auto_start=True)
        controller.create_thread(worker_function, name="Worker-4", auto_start=True)
        time.sleep(2)
    # All threads automatically stopped

    print("\n=== All examples completed ===")

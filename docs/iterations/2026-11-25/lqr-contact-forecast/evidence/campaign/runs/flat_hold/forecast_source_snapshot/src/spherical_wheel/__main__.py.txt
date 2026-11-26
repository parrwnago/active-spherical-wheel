from .cli import main

# The guard keeps worker processes started by the off-road suite from running
# the command line again when they re-import this module.
if __name__ == '__main__':
    main()

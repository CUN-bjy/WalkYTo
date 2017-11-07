# net-viz

import neat, visualize


def net_viz():


def load_checkpoint():
	p = neat.Checkpointer.restore_checkpoint(ckp_name)

	p.run(net_viz, max_iter)


if __name__ == '__main__':
	load_checkpoint()
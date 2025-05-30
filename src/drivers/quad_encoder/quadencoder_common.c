#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <lib/quad_encoder/quadencoder_ioctl.h>

#include "quadencoder_common.h"

/* Lower half operations */
static int quadencoder_setup(FAR struct qe_lowerhalf_s *lower);
static int quadencoder_shutdown(FAR struct qe_lowerhalf_s *lower);
static int quadencoder_position(FAR struct qe_lowerhalf_s *lower,
				FAR int32_t *pos);
static int quadencoder_reset(FAR struct qe_lowerhalf_s *lower);
static int quadencoder_ioctl(FAR struct qe_lowerhalf_s *lower,
			     int cmd, unsigned long arg);

/* Quad encoder operations */
static const struct qe_ops_s g_quadencoder_ops = {
	.setup     = quadencoder_setup,
	.shutdown  = quadencoder_shutdown,
	.position  = quadencoder_position,
	.reset     = quadencoder_reset,
	.ioctl     = quadencoder_ioctl
};

/* Helper functions */
static int quadencoder_interrupt_handler(int irq, void *context, void *arg)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)arg;
	uint32_t status;
	int32_t delta = 0;

	/* Read interrupt status */
	status = getreg32(priv->base_addr + QUADENCODER_STATUS_OFFSET);

	/* Handle based on mode */
	switch (priv->mode) {
	case QUADENCODER_MODE_X4:
		/* Count all edges */
		if (status & QUADENCODER_STATUS_A_EDGE) {
			delta += (status & QUADENCODER_STATUS_DIR) ? 1 : -1;
		}

		if (status & QUADENCODER_STATUS_B_EDGE) {
			delta += (status & QUADENCODER_STATUS_DIR) ? 1 : -1;
		}

		break;

	case QUADENCODER_MODE_X2:
		/* Count A and B edges */
		if (status & (QUADENCODER_STATUS_A_EDGE | QUADENCODER_STATUS_B_EDGE)) {
			delta = (status & QUADENCODER_STATUS_DIR) ? 1 : -1;
		}

		break;

	case QUADENCODER_MODE_X1:
		/* Count A edges only */
		if (status & QUADENCODER_STATUS_A_EDGE) {
			delta = (status & QUADENCODER_STATUS_DIR) ? 1 : -1;
		}

		break;
	}

	/* Update position */
	priv->position += delta;

	/* Handle index pulse if enabled */
	if (priv->index_enabled && (status & QUADENCODER_STATUS_INDEX)) {
		/* Reset position on index */
		priv->position = 0;
	}

	/* Calculate velocity */
	uint32_t current_time = up_systime_ticks();
	uint32_t dt = current_time - priv->timestamp;

	if (dt > 0) {
		priv->velocity = (priv->position - priv->last_position) * USEC_PER_SEC / dt;
		priv->last_position = priv->position;
		priv->timestamp = current_time;
	}

	/* Notify upper layer if callback registered */
	if (priv->callback) {
		priv->callback(priv->arg);
	}

	/* Clear interrupt */
	putreg32(status, priv->base_addr + QUADENCODER_CLEAR_OFFSET);

	return OK;
}

/* Common initialization */
int quadencoder_common_initialize(struct quadencoder_dev_s *dev,
				  uint8_t id,
				  uint32_t base_addr)
{
	DEBUGASSERT(dev != NULL);
	DEBUGASSERT(id < QUADENCODER_MAX_INSTANCES);

	/* Initialize device structure */
	memset(dev, 0, sizeof(struct quadencoder_dev_s));

	dev->lower.ops = &g_quadencoder_ops;
	dev->id = id;
	dev->base_addr = base_addr;
	dev->mode = QUADENCODER_MODE_X4;  /* Default to X4 mode */
	dev->resolution = 1024;           /* Default resolution */

	/* Initialize semaphore */
	nxsem_init(&dev->exclsem, 0, 1);

	/* Configure hardware */
	quadencoder_common_setup(dev);

	return OK;
}

/* Setup hardware */
int quadencoder_common_setup(struct quadencoder_dev_s *dev)
{
	uint32_t regval;
	irqstate_t flags;

	DEBUGASSERT(dev != NULL);

	flags = enter_critical_section();

	/* Reset hardware */
	putreg32(QUADENCODER_CTRL_RESET, dev->base_addr + QUADENCODER_CTRL_OFFSET);

	/* Configure mode */
	regval = getreg32(dev->base_addr + QUADENCODER_CTRL_OFFSET);
	regval &= ~QUADENCODER_CTRL_MODE_MASK;
	regval |= (dev->mode << QUADENCODER_CTRL_MODE_SHIFT);

	/* Enable filtering if configured */
	if (dev->filter_level > 0) {
		regval |= QUADENCODER_CTRL_FILTER_EN;
		regval |= (dev->filter_level << QUADENCODER_CTRL_FILTER_SHIFT);
	}

	/* Enable index if configured */
	if (dev->index_enabled) {
		regval |= QUADENCODER_CTRL_INDEX_EN;
	}

	putreg32(regval, dev->base_addr + QUADENCODER_CTRL_OFFSET);

	/* Configure interrupts */
	if (dev->irq_a > 0) {
		irq_attach(dev->irq_a, quadencoder_interrupt_handler, dev);
		up_enable_irq(dev->irq_a);
	}

	if (dev->irq_b > 0 && dev->mode >= QUADENCODER_MODE_X2) {
		irq_attach(dev->irq_b, quadencoder_interrupt_handler, dev);
		up_enable_irq(dev->irq_b);
	}

	if (dev->irq_z > 0 && dev->index_enabled) {
		irq_attach(dev->irq_z, quadencoder_interrupt_handler, dev);
		up_enable_irq(dev->irq_z);
	}

	/* Enable encoder */
	regval |= QUADENCODER_CTRL_ENABLE;
	putreg32(regval, dev->base_addr + QUADENCODER_CTRL_OFFSET);

	leave_critical_section(flags);

	return OK;
}

/* Shutdown hardware */
int quadencoder_common_shutdown(struct quadencoder_dev_s *dev)
{
	uint32_t regval;
	irqstate_t flags;

	DEBUGASSERT(dev != NULL);

	flags = enter_critical_section();

	/* Disable encoder */
	regval = getreg32(dev->base_addr + QUADENCODER_CTRL_OFFSET);
	regval &= ~QUADENCODER_CTRL_ENABLE;
	putreg32(regval, dev->base_addr + QUADENCODER_CTRL_OFFSET);

	/* Disable interrupts */
	if (dev->irq_a > 0) {
		up_disable_irq(dev->irq_a);
		irq_detach(dev->irq_a);
	}

	if (dev->irq_b > 0) {
		up_disable_irq(dev->irq_b);
		irq_detach(dev->irq_b);
	}

	if (dev->irq_z > 0) {
		up_disable_irq(dev->irq_z);
		irq_detach(dev->irq_z);
	}

	leave_critical_section(flags);

	return OK;
}

/* Get current position */
int32_t quadencoder_common_position(struct quadencoder_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);
	return dev->position;
}

/* Reset position */
int quadencoder_common_reset(struct quadencoder_dev_s *dev)
{
	irqstate_t flags;

	DEBUGASSERT(dev != NULL);

	flags = enter_critical_section();
	dev->position = 0;
	dev->last_position = 0;
	leave_critical_section(flags);

	return OK;
}

/* Set counting mode */
int quadencoder_common_setmode(struct quadencoder_dev_s *dev,
			       enum quadencoder_mode_e mode)
{
	uint32_t regval;
	irqstate_t flags;

	DEBUGASSERT(dev != NULL);

	if (mode != QUADENCODER_MODE_X1 &&
	    mode != QUADENCODER_MODE_X2 &&
	    mode != QUADENCODER_MODE_X4) {
		return -EINVAL;
	}

	flags = enter_critical_section();

	dev->mode = mode;

	/* Update hardware configuration */
	regval = getreg32(dev->base_addr + QUADENCODER_CTRL_OFFSET);
	regval &= ~QUADENCODER_CTRL_MODE_MASK;
	regval |= (mode << QUADENCODER_CTRL_MODE_SHIFT);
	putreg32(regval, dev->base_addr + QUADENCODER_CTRL_OFFSET);

	leave_critical_section(flags);

	return OK;
}

/* Lower half operations implementation */
static int quadencoder_setup(FAR struct qe_lowerhalf_s *lower)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)lower;
	return quadencoder_common_setup(priv);
}

static int quadencoder_shutdown(FAR struct qe_lowerhalf_s *lower)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)lower;
	return quadencoder_common_shutdown(priv);
}

static int quadencoder_position(FAR struct qe_lowerhalf_s *lower,
				FAR int32_t *pos)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)lower;

	if (!pos) {
		return -EINVAL;
	}

	*pos = quadencoder_common_position(priv);
	return OK;
}

static int quadencoder_reset(FAR struct qe_lowerhalf_s *lower)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)lower;
	return quadencoder_common_reset(priv);
}

static int quadencoder_ioctl(FAR struct qe_lowerhalf_s *lower,
			     int cmd, unsigned long arg)
{
	struct quadencoder_dev_s *priv = (struct quadencoder_dev_s *)lower;
	int ret = OK;

	switch (cmd) {
	case QEIOC_SETMODE:
		ret = quadencoder_common_setmode(priv, (enum quadencoder_mode_e)arg);
		break;

	case QEIOC_GETVELOCITY:
		*(int32_t *)arg = priv->velocity;
		break;

	case QEIOC_SETRESOLUTION:
		priv->resolution = (uint32_t)arg;
		break;

	case QEIOC_GETRESOLUTION:
		*(uint32_t *)arg = priv->resolution;
		break;

	case QEIOC_SETFILTER:
		priv->filter_level = (uint8_t)arg;
		break;

	case QEIOC_ENABLEINDEX:
		priv->index_enabled = true;
		break;

	case QEIOC_DISABLEINDEX:
		priv->index_enabled = false;
		break;

	case QEIOC_GETANGLE:
		*(float *)arg = (float)priv->position * 360.0f / (float)priv->resolution;
		break;

	case QEIOC_GETRPM:
		/* Calculate RPM from velocity */
		*(int32_t *)arg = (priv->velocity * 60) / priv->resolution;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

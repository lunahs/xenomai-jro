/**
 * @file
 * Analogy driver for Sensoray Model 626 board
 *
 * Copyright (C) 2014 Wojciech Domski <wojciech.domski@gmail.com>
 * Copyright (C) 2014 Mariusz Janiak  <mariusz.janiak@pwr.wroc.pl>
 *
 * Derived from comedi:
 * Copyright (C) 2000 David A. Schleef <ds@schleef.org>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
/*
 * Original code comes from comedi linux-next staging driver
 * Board documentation: http://www.sensoray.com/products/626.htm
 * Everything should work as in comedi.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <asm/byteorder.h>
#include <linux/pci.h>

#include <rtdm/analogy/driver.h>
#include <rtdm/analogy/device.h>

#include "s626.h"
#define MAX_SPEED 200000 		/* in nanoseconds */
#define MIN_SPEED 2000000000  		/* in nanoseconds */

static LIST_HEAD(s626_list);

/*  TrimDac LogicalChan-to-PhysicalChan mapping table. */
static uint8_t trimchan[] = { 10, 9, 8, 3, 2, 7, 6, 1, 0, 5, 4 };

/*  TrimDac LogicalChan-to-EepromAdrs mapping table. */
static uint8_t trimadrs[] = {
	0x40, 0x41, 0x42, 0x50, 0x51, 0x52, 0x53, 0x60, 0x61, 0x62, 0x63 };

/*
 * For devices with vendor:device id == 0x1131:0x7146 you must specify
 * also subvendor:subdevice ids, because otherwise it will conflict with
 * Philips SAA7146 media/dvb based cards.
 */
static struct pci_device_id s626_pci_table[] = {
	[0] = { 0x1131, 0x7146, 0x6000,0x0626, 0, 0, 0 },
	[1] = { 0x1131, 0x7146, 0x6000, 626, 0, 0, 0 },
	[2] = { 0, }
};

MODULE_DEVICE_TABLE(pci, s626_pci_table);

static struct pci_driver drv_pci_s626 = {
	.name = "S626",
	.id_table = s626_pci_table,
	.probe = s626_pci_probe,
	.remove = s626_pci_remove,
};

static struct s626_subd_dio_private s626_subd_dio_private_A = {
	.rdd_in = LP_RDDINA,
	.wrd_out = LP_WRDOUTA,
	.rd_edg_sel = LP_RDEDGSELA,
	.wr_edg_sel = LP_WREDGSELA,
	.rd_cap_sel = LP_RDCAPSELA,
	.wr_cap_sel = LP_WRCAPSELA,
	.rd_cap_flg = LP_RDCAPFLGA,
	.rd_int_sel = LP_RDINTSELA,
	.wr_int_sel = LP_WRINTSELA,
	.io_bits = 0,
	.state = 0,
};

static struct s626_subd_dio_private s626_subd_dio_private_B = {
	.rdd_in = LP_RDDINB,
	.wrd_out = LP_WRDOUTB,
	.rd_edg_sel = LP_RDEDGSELB,
	.wr_edg_sel = LP_WREDGSELB,
	.rd_cap_sel = LP_RDCAPSELB,
	.wr_cap_sel = LP_WRCAPSELB,
	.rd_cap_flg = LP_RDCAPFLGB,
	.rd_int_sel = LP_RDINTSELB,
	.wr_int_sel = LP_WRINTSELB,
	.io_bits = 0,
	.state = 0,
};

static struct s626_subd_dio_private s626_subd_dio_private_C = {
	.rdd_in = LP_RDDINC,
	.wrd_out = LP_WRDOUTC,
	.rd_edg_sel = LP_RDEDGSELC,
	.wr_edg_sel = LP_WREDGSELC,
	.rd_cap_sel = LP_RDCAPSELC,
	.wr_cap_sel = LP_WRCAPSELC,
	.rd_cap_flg = LP_RDCAPFLGC,
	.rd_int_sel = LP_RDINTSELC,
	.wr_int_sel = LP_WRINTSELC,
	.io_bits = 0,
	.state = 0,
};

/* ******  PRIVATE COUNTER FUNCTIONS ****** */

/*  Reset a counter's index and overflow event capture flags. */

static void reset_cap_flags_a(struct s626_struct * s626ptr, struct enc_private *k)
{
	debi_replace(s626ptr, k->my_crb, (uint16_t) (~CRBMSK_INTCTRL),
		     CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A);
}

static void reset_cap_flags_b(struct s626_struct * s626ptr, struct enc_private *k)
{
	debi_replace(s626ptr, k->my_crb, (uint16_t) (~CRBMSK_INTCTRL),
		     CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B);
}

/*  Return counter setup in a format (COUNTER_SETUP) that is consistent */
/*  for both A and B counters. */

static uint16_t get_mode_a(struct s626_struct * s626ptr, struct enc_private *k)
{
	register uint16_t cra;
	register uint16_t crb;
	register uint16_t setup;

	/*  Fetch CRA and CRB register images. */
	cra = debi_read(s626ptr, k->my_cra);
	crb = debi_read(s626ptr, k->my_crb);

	/*  Populate the standardized counter setup bit fields.  Note: */
	/*  IndexSrc is restricted to ENC_X or IndxPol. */
	/*  LoadSrc  = LoadSrcA. */
	/*  LatchSrc = LatchSrcA. */
	/*  IntSrc   = IntSrcA. */
	/*  IndxSrc  = IndxSrcA<1>. */
	/*  IndxPol  = IndxPolA. */
	/*  ClkEnab  = ClkEnabA. */
	setup = ((cra & STDMSK_LOADSRC) |
		 ((crb << (STDBIT_LATCHSRC - CRBBIT_LATCHSRC)) & STDMSK_LATCHSRC) |
		 ((cra << (STDBIT_INTSRC - CRABIT_INTSRC_A)) & STDMSK_INTSRC) |
		 ((cra << (STDBIT_INDXSRC - (CRABIT_INDXSRC_A + 1))) & STDMSK_INDXSRC) |
		 ((cra >> (CRABIT_INDXPOL_A - STDBIT_INDXPOL)) & STDMSK_INDXPOL) |
		 ((crb >> (CRBBIT_CLKENAB_A - STDBIT_CLKENAB)) & STDMSK_CLKENAB));

	/*  Adjust mode-dependent parameters. */
	/*  If Timer mode (ClkSrcA<1> == 1): */
	/*    Indicate Timer mode. */
	/*    Set ClkPol to indicate count direction (ClkSrcA<0>). */
	/*    ClkMult must be 1x in Timer mode. */
	if (cra & (2 << CRABIT_CLKSRC_A))
		setup |= ((CLKSRC_TIMER << STDBIT_CLKSRC) |
			  ((cra << (STDBIT_CLKPOL - CRABIT_CLKSRC_A)) & STDMSK_CLKPOL) |
			  (MULT_X1 << STDBIT_CLKMULT));

	else {
		/*  If Counter mode (ClkSrcA<1> == 0): */
		/*    Indicate Counter mode. */
		/*    Pass through ClkPol. */
		/*    Force ClkMult to 1x if not legal, else pass through. */

		setup |= ((CLKSRC_COUNTER << STDBIT_CLKSRC) |
			  ((cra >> (CRABIT_CLKPOL_A - STDBIT_CLKPOL)) & STDMSK_CLKPOL) |
			  (((cra & CRAMSK_CLKMULT_A) == (MULT_X0 << CRABIT_CLKMULT_A)) ?
			   (MULT_X1 << STDBIT_CLKMULT) :
			   ((cra >> (CRABIT_CLKMULT_A - STDBIT_CLKMULT)) & STDMSK_CLKMULT)));
	}

	return setup;
}

static uint16_t get_mode_b(struct s626_struct * s626ptr, struct enc_private *k)
{
	register uint16_t cra;
	register uint16_t crb;
	register uint16_t setup;

	/*  Fetch CRA and CRB register images. */
	cra = debi_read(s626ptr, k->my_cra);
	crb = debi_read(s626ptr, k->my_crb);

	/*  Populate the standardized counter setup bit fields.  Note: */
	/*  IndexSrc is restricted to ENC_X or IndxPol. */
	/*  IntSrc   = IntSrcB. */
	/*  LatchSrc = LatchSrcB. */
	/*  LoadSrc  = LoadSrcB. */
	/*  IndxPol  = IndxPolB. */
	/*  ClkEnab  = ClkEnabB. */
	/*  IndxSrc  = IndxSrcB<1>. */
	setup = (((crb << (STDBIT_INTSRC - CRBBIT_INTSRC_B)) & STDMSK_INTSRC)
		 | ((crb << (STDBIT_LATCHSRC - CRBBIT_LATCHSRC)) & STDMSK_LATCHSRC)
		 | ((crb << (STDBIT_LOADSRC - CRBBIT_LOADSRC_B)) & STDMSK_LOADSRC)
		 | ((crb << (STDBIT_INDXPOL - CRBBIT_INDXPOL_B)) & STDMSK_INDXPOL)
		 | ((crb >> (CRBBIT_CLKENAB_B - STDBIT_CLKENAB)) & STDMSK_CLKENAB)
		 | ((cra >> ((CRABIT_INDXSRC_B + 1) - STDBIT_INDXSRC)) & STDMSK_INDXSRC));

	/*  Adjust mode-dependent parameters. */
	/*  If Extender mode (ClkMultB == MULT_X0): */
	/*    Indicate Extender mode. */
	/*    Indicate multiplier is 1x. */
	/*    Set ClkPol equal to Timer count direction (ClkSrcB<0>). */
	if ((crb & CRBMSK_CLKMULT_B) == (MULT_X0 << CRBBIT_CLKMULT_B))
		setup |= ((CLKSRC_EXTENDER << STDBIT_CLKSRC)
			  | (MULT_X1 << STDBIT_CLKMULT)
			  | ((cra >> (CRABIT_CLKSRC_B - STDBIT_CLKPOL)) & STDMSK_CLKPOL));

	/*  If Timer mode (ClkSrcB<1> == 1): */
	/*    Indicate Timer mode. */
	/*    Indicate multiplier is 1x. */
	/*    Set ClkPol equal to Timer count direction (ClkSrcB<0>). */
	else if (cra & (2 << CRABIT_CLKSRC_B))
		setup |= ((CLKSRC_TIMER << STDBIT_CLKSRC)
			  | (MULT_X1 << STDBIT_CLKMULT)
			  | ((cra >> (CRABIT_CLKSRC_B - STDBIT_CLKPOL)) & STDMSK_CLKPOL));

	/*  If Counter mode (ClkSrcB<1> == 0): */
	/*    Indicate Timer mode. */
	/*    Clock multiplier is passed through. */
	/*    Clock polarity is passed through. */
	else
		setup |= ((CLKSRC_COUNTER << STDBIT_CLKSRC)
			  | ((crb >> (CRBBIT_CLKMULT_B - STDBIT_CLKMULT)) & STDMSK_CLKMULT)
			  | ((crb << (STDBIT_CLKPOL - CRBBIT_CLKPOL_B)) & STDMSK_CLKPOL));

	return setup;
}

/*
 * Set the operating mode for the specified counter.  The setup
 * parameter is treated as a COUNTER_SETUP data type.  The following
 * parameters are programmable (all other parms are ignored): ClkMult,
 * ClkPol, ClkEnab, IndexSrc, IndexPol, LoadSrc.
 */

static void set_mode_a(struct s626_struct * s626ptr, struct enc_private *k,
		       uint16_t Setup, uint16_t DisableIntSrc)
{
	struct s626_priv *devpriv = s626ptr->private;
	register uint16_t cra;
	register uint16_t crb;
	register uint16_t setup = Setup; /*  Cache the Standard Setup. */

	/*  Initialize CRA and CRB images. */
	/*  preload trigger is passed through. */
	/*  IndexSrc is restricted to ENC_X or IndxPol. */
	cra = ((setup & CRAMSK_LOADSRC_A) |
	       ((setup & STDMSK_INDXSRC) >> (STDBIT_INDXSRC - (CRABIT_INDXSRC_A + 1))));

	/*  Reset any pending CounterA event captures. */
	/*  Clock enable is passed through. */
	crb = (CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A |
	       ((setup & STDMSK_CLKENAB) << (CRBBIT_CLKENAB_A - STDBIT_CLKENAB)));

	/*  Force IntSrc to Disabled if DisableIntSrc is asserted. */
	if (!DisableIntSrc)
		cra |= ((setup & STDMSK_INTSRC) >> (STDBIT_INTSRC - CRABIT_INTSRC_A));

	/*  Populate all mode-dependent attributes of CRA & CRB images. */
	switch ((setup & STDMSK_CLKSRC) >> STDBIT_CLKSRC) {
	case CLKSRC_EXTENDER:
		/*  Extender Mode: Force to Timer mode */
		/*  (Extender valid only for B counters). */
	case CLKSRC_TIMER:
		/*  Timer Mode: */
		/*    ClkSrcA<1> selects system clock */
		/*      with count direction (ClkSrcA<0>) obtained from ClkPol. */
		/*    ClkPolA behaves as always-on clock enable. */
		/*    ClkMult must be 1x. */
		cra |= ((2 << CRABIT_CLKSRC_A) |
			((setup & STDMSK_CLKPOL) >> (STDBIT_CLKPOL - CRABIT_CLKSRC_A)) |
			(1 << CRABIT_CLKPOL_A) |
			(MULT_X1 << CRABIT_CLKMULT_A));
		break;

	default:
		/*  Counter Mode: */
		/*    Select ENC_C and ENC_D as clock/direction inputs. */
		/*    Clock polarity is passed through. */
		/*    Force multiplier to x1 if not legal, otherwise pass through. */
		cra |= (CLKSRC_COUNTER |
			((setup & STDMSK_CLKPOL) << (CRABIT_CLKPOL_A - STDBIT_CLKPOL)) |
			(((setup & STDMSK_CLKMULT) == (MULT_X0 << STDBIT_CLKMULT)) ?
			 (MULT_X1 << CRABIT_CLKMULT_A) :
			 ((setup & STDMSK_CLKMULT)<< (CRABIT_CLKMULT_A - STDBIT_CLKMULT))));
		break;
	}

	/*  Force positive index polarity if IndxSrc is software-driven only, */
	/*  otherwise pass it through. */
	if (~setup & STDMSK_INDXSRC)
		cra |= ((setup & STDMSK_INDXPOL) << (CRABIT_INDXPOL_A - STDBIT_INDXPOL));

	/*  If IntSrc has been forced to Disabled, update the MISC2 interrupt */
	/*  enable mask to indicate the counter interrupt is disabled. */
	if (DisableIntSrc)
		devpriv->CounterIntEnabs &= ~k->my_event_bits[3];

	/*  While retaining CounterB and LatchSrc configurations, program the */
	/*  new counter operating mode. */
	debi_replace(s626ptr, k->my_cra, CRAMSK_INDXSRC_B | CRAMSK_CLKSRC_B, cra);
	debi_replace(s626ptr, k->my_crb, (uint16_t) (~(CRBMSK_INTCTRL | CRBMSK_CLKENAB_A)), crb);
}

static void set_mode_b(struct s626_struct * s626ptr, struct enc_private *k,
		       uint16_t Setup, uint16_t DisableIntSrc)
{
	struct s626_priv *devpriv = s626ptr->private;
	register uint16_t cra;
	register uint16_t crb;
	register uint16_t setup = Setup; /*  Cache the Standard Setup. */

	/*  Initialize CRA and CRB images. */
	/*  IndexSrc field is restricted to ENC_X or IndxPol. */
	cra = ((setup & STDMSK_INDXSRC) << ((CRABIT_INDXSRC_B + 1) - STDBIT_INDXSRC));

	/*  Reset event captures and disable interrupts. */
	/*  Clock enable is passed through. */
	/*  preload trigger source is passed through. */
	crb = (CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B |
	       ((setup & STDMSK_CLKENAB) << (CRBBIT_CLKENAB_B - STDBIT_CLKENAB)) |
	       ((setup & STDMSK_LOADSRC) >> (STDBIT_LOADSRC - CRBBIT_LOADSRC_B)));

	/*  Force IntSrc to Disabled if DisableIntSrc is asserted. */
	if (!DisableIntSrc)
		crb |= ((setup & STDMSK_INTSRC) >> (STDBIT_INTSRC - CRBBIT_INTSRC_B));

	/*  Populate all mode-dependent attributes of CRA & CRB images. */
	switch ((setup & STDMSK_CLKSRC) >> STDBIT_CLKSRC) {
	case CLKSRC_TIMER:
		/*  Timer Mode: */
		/*    ClkSrcB<1> selects system clock */
		/*      with direction (ClkSrcB<0>) obtained from ClkPol. */
		cra |= ((2 << CRABIT_CLKSRC_B) |
			((setup & STDMSK_CLKPOL) << (CRABIT_CLKSRC_B - STDBIT_CLKPOL)));

		/*    ClkPolB behaves as always-on clock enable. */
		/*    ClkMultB must be 1x. */
		crb |= ((1 << CRBBIT_CLKPOL_B) |
			(MULT_X1 << CRBBIT_CLKMULT_B));
		break;

	case CLKSRC_EXTENDER:
		/*  Extender Mode: */
		/*    ClkSrcB source is OverflowA (same as "timer") */
		/*      with direction obtained from ClkPol. */
		cra |= ((2 << CRABIT_CLKSRC_B) |
			((setup & STDMSK_CLKPOL) << (CRABIT_CLKSRC_B - STDBIT_CLKPOL)));

		/*    ClkPolB controls IndexB -- always set to active. */
		/*    ClkMultB selects OverflowA as the clock source. */
		crb |= ((1 << CRBBIT_CLKPOL_B) |
			(MULT_X0 << CRBBIT_CLKMULT_B));
		break;

	default:
		/*  Counter Mode: */
		/*    Select ENC_C and ENC_D as clock/direction inputs. */
		/*    ClkPol is passed through. */
		cra |= (CLKSRC_COUNTER << CRABIT_CLKSRC_B);

		/*    Force ClkMult to x1 if not legal, otherwise pass through. */
		crb |= (((setup & STDMSK_CLKPOL) >> (STDBIT_CLKPOL - CRBBIT_CLKPOL_B)) |
			(((setup & STDMSK_CLKMULT) == (MULT_X0 << STDBIT_CLKMULT)) ?
			 (MULT_X1 << CRBBIT_CLKMULT_B) :
			 ((setup & STDMSK_CLKMULT)<< (CRBBIT_CLKMULT_B - STDBIT_CLKMULT))));
		break;
	}

	/*  Force positive index polarity if IndxSrc is software-driven only, */
	/*  otherwise pass it through. */
	if (~setup & STDMSK_INDXSRC)
		crb |= ((setup & STDMSK_INDXPOL) >> (STDBIT_INDXPOL - CRBBIT_INDXPOL_B));

	/*  If IntSrc has been forced to Disabled, update the MISC2 interrupt */
	/*  enable mask to indicate the counter interrupt is disabled. */
	if (DisableIntSrc)
		devpriv->CounterIntEnabs &= ~k->my_event_bits[3];

	/*  While retaining CounterA and LatchSrc configurations, program the */
	/*  new counter operating mode. */
	debi_replace(s626ptr, k->my_cra, (uint16_t) (~(CRAMSK_INDXSRC_B | CRAMSK_CLKSRC_B)), cra);
	debi_replace(s626ptr, k->my_crb, CRBMSK_CLKENAB_A | CRBMSK_LATCHSRC, crb);
}

/*  Return/set a counter's enable.  enab: 0=always enabled, 1=enabled by index. */

static void set_enable_a(struct s626_struct * s626ptr, struct enc_private *k,
			 uint16_t enab)
{
	debi_replace(s626ptr, k->my_crb,
		     (uint16_t) (~(CRBMSK_INTCTRL | CRBMSK_CLKENAB_A)),
		     (uint16_t) (enab << CRBBIT_CLKENAB_A));
}

static void set_enable_b(struct s626_struct * s626ptr, struct enc_private *k,
			 uint16_t enab)
{
	debi_replace(s626ptr, k->my_crb,
		     (uint16_t) (~(CRBMSK_INTCTRL | CRBMSK_CLKENAB_B)),
		     (uint16_t) (enab << CRBBIT_CLKENAB_B));
}

static uint16_t get_enable_a(struct s626_struct * s626ptr, struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_crb) >> CRBBIT_CLKENAB_A) & 1;
}

static uint16_t get_enable_b(struct s626_struct * s626ptr, struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_crb) >> CRBBIT_CLKENAB_B) & 1;
}


/*
 * Return/set the event that will trigger transfer of the preload
 * register into the counter.  0=ThisCntr_Index, 1=ThisCntr_Overflow,
 * 2=OverflowA (B counters only), 3=disabled.
 */

static void set_load_trig_a(struct s626_struct * s626ptr, struct enc_private *k,
			    uint16_t Trig)
{
	debi_replace(s626ptr, k->my_cra, (uint16_t) (~CRAMSK_LOADSRC_A),
		     (uint16_t) (Trig << CRABIT_LOADSRC_A));
}

static void set_load_trig_b(struct s626_struct * s626ptr, struct enc_private *k,
			    uint16_t Trig)
{
	debi_replace(s626ptr, k->my_crb,
		     (uint16_t) (~(CRBMSK_LOADSRC_B | CRBMSK_INTCTRL)),
		     (uint16_t) (Trig << CRBBIT_LOADSRC_B));
}

static uint16_t get_load_trig_a(struct s626_struct * s626ptr,
				struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_cra) >> CRABIT_LOADSRC_A) & 3;
}

static uint16_t get_load_trig_b(struct s626_struct * s626ptr,
				struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_crb) >> CRBBIT_LOADSRC_B) & 3;
}

/* Return/set counter interrupt source and clear any captured
 * index/overflow events.  IntSource: 0=Disabled, 1=OverflowOnly,
 * 2=IndexOnly, 3=IndexAndOverflow.
 */

static void set_int_src_a(struct s626_struct * s626ptr, struct enc_private *k,
			  uint16_t IntSource)
{
	struct s626_priv *devpriv = s626ptr->private;

	/*  Reset any pending counter overflow or index captures. */
	debi_replace(s626ptr, k->my_crb, (uint16_t) (~CRBMSK_INTCTRL),
		     CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A);

	/*  Program counter interrupt source. */
	debi_replace(s626ptr, k->my_cra, ~CRAMSK_INTSRC_A,
		     (uint16_t) (IntSource << CRABIT_INTSRC_A));

	/*  Update MISC2 interrupt enable mask. */
	devpriv->CounterIntEnabs = (devpriv->CounterIntEnabs & ~k->my_event_bits[3])
				   | k->my_event_bits[IntSource];
}

static void set_int_src_b(struct s626_struct * s626ptr, struct enc_private *k,
			  uint16_t IntSource)
{
	struct s626_priv *devpriv = s626ptr->private;
	uint16_t crb;

	/*  Cache writeable CRB register image. */
	crb = debi_read(s626ptr, k->my_crb) & ~CRBMSK_INTCTRL;

	/*  Reset any pending counter overflow or index captures. */
	debi_write(s626ptr, k->my_crb,
		   crb | CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B);

	/*  Program counter interrupt source. */
	debi_write(s626ptr, k->my_crb,
		   (crb & ~CRBMSK_INTSRC_B) | (IntSource << CRBBIT_INTSRC_B));

	/*  Update MISC2 interrupt enable mask. */
	devpriv->CounterIntEnabs = (devpriv->CounterIntEnabs & ~k->my_event_bits[3])
				   | k->my_event_bits[IntSource];
}

static uint16_t get_int_src_a(struct s626_struct * s626ptr, struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_cra) >> CRABIT_INTSRC_A) & 3;
}

static uint16_t get_int_src_b(struct s626_struct * s626ptr, struct enc_private *k)
{
	return(debi_read(s626ptr, k->my_crb) >> CRBBIT_INTSRC_B) & 3;
}

/*  Generate an index pulse. */

static void pulse_index_a(struct s626_struct * s626ptr, struct enc_private *k)
{
	register uint16_t cra;

	cra = debi_read(s626ptr, k->my_cra); /*  Pulse index. */
	debi_write(s626ptr, k->my_cra, (uint16_t) (cra ^ CRAMSK_INDXPOL_A));
	debi_write(s626ptr, k->my_cra, cra);
}

static void pulse_index_b(struct s626_struct * s626ptr, struct enc_private *k)
{
	register uint16_t crb;

	crb = debi_read(s626ptr, k->my_crb) & ~CRBMSK_INTCTRL; /*  Pulse index. */
	debi_write(s626ptr, k->my_crb, (uint16_t) (crb ^ CRBMSK_INDXPOL_B));
	debi_write(s626ptr, k->my_crb, crb);
}

static struct enc_private enc_private_data_init[] = {
	[0] = {
		.get_enable = get_enable_a,
		.get_int_src = get_int_src_a,
		.get_load_trig = get_load_trig_a,
		.get_mode = get_mode_a,
		.pulse_index = pulse_index_a,
		.set_enable = set_enable_a,
		.set_int_src = set_int_src_a,
		.set_load_trig = set_load_trig_a,
		.set_mode = set_mode_a,
		.reset_cap_flags = reset_cap_flags_a,
		.my_cra = LP_CR0A,
		.my_crb = LP_CR0B,
		.my_latch_lsw = LP_CNTR0ALSW,
		.my_event_bits = EVBITS(0),
	},
	[1] = {
		.get_enable = get_enable_a,
		.get_int_src = get_int_src_a,
		.get_load_trig = get_load_trig_a,
		.get_mode = get_mode_a,
		.pulse_index = pulse_index_a,
		.set_enable = set_enable_a,
		.set_int_src = set_int_src_a,
		.set_load_trig = set_load_trig_a,
		.set_mode = set_mode_a,
		.reset_cap_flags = reset_cap_flags_a,
		.my_cra = LP_CR1A,
		.my_crb = LP_CR1B,
		.my_latch_lsw = LP_CNTR1ALSW,
		.my_event_bits = EVBITS(1), },
	[2] = {
		.get_enable = get_enable_a,
		.get_int_src = get_int_src_a,
		.get_load_trig = get_load_trig_a,
		.get_mode = get_mode_a,
		.pulse_index = pulse_index_a,
		.set_enable = set_enable_a,
		.set_int_src = set_int_src_a,
		.set_load_trig = set_load_trig_a,
		.set_mode = set_mode_a,
		.reset_cap_flags = reset_cap_flags_a,
		.my_cra = LP_CR2A,
		.my_crb = LP_CR2B,
		.my_latch_lsw = LP_CNTR2ALSW,
		.my_event_bits = EVBITS(2), },
	[3] = {
		.get_enable = get_enable_b,
		.get_int_src = get_int_src_b,
		.get_load_trig = get_load_trig_b,
		.get_mode = get_mode_b,
		.pulse_index = pulse_index_b,
		.set_enable = set_enable_b,
		.set_int_src = set_int_src_b,
		.set_load_trig = set_load_trig_b,
		.set_mode = set_mode_b,
		.reset_cap_flags = reset_cap_flags_b,
		.my_cra = LP_CR0A,
		.my_crb = LP_CR0B,
		.my_latch_lsw = LP_CNTR0BLSW,
		.my_event_bits = EVBITS(3), },
	[4] = {
		.get_enable = get_enable_b,
		.get_int_src = get_int_src_b,
		.get_load_trig = get_load_trig_b,
		.get_mode = get_mode_b,
		.pulse_index = pulse_index_b,
		.set_enable = set_enable_b,
		.set_int_src = set_int_src_b,
		.set_load_trig = set_load_trig_b,
		.set_mode = set_mode_b,
		.reset_cap_flags = reset_cap_flags_b,
		.my_cra = LP_CR1A,
		.my_crb = LP_CR1B,
		.my_latch_lsw = LP_CNTR1BLSW,
		.my_event_bits = EVBITS(4), },
	[5] = {
		.get_enable =get_enable_b,
		.get_int_src = get_int_src_b,
		.get_load_trig = get_load_trig_b,
		.get_mode = get_mode_b,
		.pulse_index = pulse_index_b,
		.set_enable = set_enable_b,
		.set_int_src = set_int_src_b,
		.set_load_trig = set_load_trig_b,
		.set_mode = set_mode_b,
		.reset_cap_flags = reset_cap_flags_b,
		.my_cra = LP_CR2A,
		.my_crb = LP_CR2B,
		.my_latch_lsw = LP_CNTR2BLSW,
		.my_event_bits = EVBITS(5), },
};

static void close_dmab(struct s626_struct * s626ptr, struct bufferDMA *pdma,
		       size_t bsize)
{
	struct pci_dev *pcidev = s626ptr->pcidev;
	void *vbptr;
	dma_addr_t vpptr;

	if (pdma == NULL)
		return;
	/* find the matching allocation from the board struct */

	vbptr = pdma->LogicalBase;
	vpptr = pdma->PhysicalBase;
	if (vbptr) {
		pci_free_consistent(pcidev, bsize, vbptr, vpptr);
		pdma->LogicalBase = NULL;
		pdma->PhysicalBase = 0;
	}
}

/*  Initialize the DEBI interface for all transfers. */

static uint16_t debi_read(struct s626_struct * s626ptr, uint16_t addr)
{
	struct s626_priv *devpriv = s626ptr->private;
	uint16_t retval;

	/*  Set up DEBI control register value in shadow RAM. */
	WR7146(P_DEBICMD, DEBI_CMD_RDWORD | addr);

	/*  Execute the DEBI transfer. */
	debi_transfer(s626ptr);

	/*  Fetch target register value. */
	retval = (uint16_t) RR7146(P_DEBIAD);

	/*  Return register value. */
	return retval;
}

/*  Write a value to a gate array register. */
static void debi_write(struct s626_struct * s626ptr, uint16_t addr,
		       uint16_t wdata)
{
	struct s626_priv *devpriv = s626ptr->private;

	/*  Set up DEBI control register value in shadow RAM. */
	WR7146(P_DEBICMD, DEBI_CMD_WRWORD | addr);
	WR7146(P_DEBIAD, wdata);

	/*  Execute the DEBI transfer. */
	debi_transfer(s626ptr);
}

/*  Execute a DEBI transfer: must be called from within a  critical section. */
static void debi_transfer(struct s626_struct * s626ptr)
{
	struct s626_priv *devpriv = s626ptr->private;

	/*  Initiate upload of shadow RAM to DEBI control register. */
	MC_ENABLE(P_MC2, MC2_UPLD_DEBI);

	/*  Wait for completion of upload from shadow RAM to DEBI control */
	/*  register. */
	while (!MC_TEST(P_MC2, MC2_UPLD_DEBI))
		rtdm_task_sleep(2000);

	/*  Wait until DEBI transfer is done. */
	while (RR7146(P_PSR) & PSR_DEBI_S)
		rtdm_task_sleep(2000);
}

static void debi_replace(struct s626_struct * s626ptr, uint16_t addr,
			 uint16_t mask, uint16_t wdata)
{
	struct s626_priv *devpriv = s626ptr->private;

	/*  Copy target gate array register into P_DEBIAD register. */
	WR7146(P_DEBICMD, DEBI_CMD_RDWORD | addr);
	/* Set up DEBI control reg value in shadow RAM. */
	debi_transfer(s626ptr);	/*  Execute the DEBI Read transfer. */

	/*  Write back the modified image. */
	WR7146(P_DEBICMD, DEBI_CMD_WRWORD | addr);
	/* Set up DEBI control reg value in shadow  RAM. */

	WR7146(P_DEBIAD, wdata | ((uint16_t) RR7146(P_DEBIAD) & mask));
	/* Modify the register image. */
	debi_transfer(s626ptr);	/*  Execute the DEBI Write transfer. */
}

/* **************  EEPROM ACCESS FUNCTIONS  ************** */

static uint32_t i2c_handshake(struct s626_struct * s626ptr, uint32_t val)
{
	struct s626_priv *devpriv = s626ptr->private;

	/*  Write I2C command to I2C Transfer Control shadow register. */
	WR7146(P_I2CCTRL, val);

	/*  Upload I2C shadow registers into working registers and wait for */
	/*  upload confirmation. */

	MC_ENABLE(P_MC2, MC2_UPLD_IIC);
	while (!MC_TEST(P_MC2, MC2_UPLD_IIC))
		rtdm_task_sleep(2000);

	/*  Wait until I2C bus transfer is finished or an error occurs. */
	while ((RR7146(P_I2CCTRL) & (I2C_BUSY | I2C_ERR)) == I2C_BUSY)
		rtdm_task_sleep(2000);

	/*  Return non-zero if I2C error occurred. */
	return RR7146(P_I2CCTRL) & I2C_ERR;

}

/*  Read uint8_t from EEPROM. */
static uint8_t i2c_read(struct s626_struct * s626ptr, uint8_t addr)
{
	struct s626_priv *devpriv = s626ptr->private;
	uint8_t rtnval;

	/*  Send EEPROM target address. */
	if (i2c_handshake(s626ptr, I2C_B2(I2C_ATTRSTART, I2CW)
			  /* Byte2 = I2C command: write to I2C EEPROM  device. */
			  | I2C_B1(I2C_ATTRSTOP, addr)
			  /* Byte1 = EEPROM internal target address. */
			  | I2C_B0(I2C_ATTRNOP, 0))) { /*  Byte0 = Not sent. */
		/*  Abort function and declare error if handshake failed. */
		return 0;
	}
	/*  Execute EEPROM read. */
	if (i2c_handshake(s626ptr, I2C_B2(I2C_ATTRSTART, I2CR)

			  /*  Byte2 = I2C */
			  /*  command: read */
			  /*  from I2C EEPROM */
			  /*  device. */
			  | I2C_B1(I2C_ATTRSTOP, 0)

			  /*  Byte1 receives */
			  /*  uint8_t from */
			  /*  EEPROM. */
			  | I2C_B0(I2C_ATTRNOP, 0))) { /*  Byte0 = Not  sent. */

		/*  Abort function and declare error if handshake failed. */
		return 0;
	}
	/*  Return copy of EEPROM value. */
	rtnval = (uint8_t) (RR7146(P_I2CCTRL) >> 16);
	return rtnval;
}

static void write_misc2(struct s626_struct * s626ptr, uint16_t NewImage)
{
	debi_write(s626ptr, LP_MISC1, MISC1_WENABLE); /*  enab writes to */
	/*  MISC2 register. */
	debi_write(s626ptr, LP_WRMISC2, NewImage); /*  Write new image to MISC2. */
	debi_write(s626ptr, LP_MISC1, MISC1_WDISABLE); /*  Disable writes to MISC2. */
}

/*  Write value into counter preload register. */
static void preload(struct s626_struct * s626ptr, struct enc_private *k,
		    uint32_t value)
{
	debi_write(s626ptr, (uint16_t) (k->my_latch_lsw), (uint16_t) value);
	debi_write(s626ptr, (uint16_t) (k->my_latch_lsw + 2), (uint16_t) (value >> 16));
}

/* Private helper function: Transmit serial data to DAC via Audio
 * channel 2.  Assumes: (1) TSL2 slot records initialized, and (2)
 * Dacpol contains valid target image.
 */
static void send_adc(struct s626_struct * s626ptr, uint32_t val)
{
	struct s626_priv *devpriv = s626ptr->private;

	/* START THE SERIAL CLOCK RUNNING ------------- */

	/* Assert DAC polarity control and enable gating of DAC serial clock
	 * and audio bit stream signals.  At this point in time we must be
	 * assured of being in time slot 0.  If we are not in slot 0, the
	 * serial clock and audio stream signals will be disabled; this is
	 * because the following debi_write statement (which enables signals
	 * to be passed through the gate array) would execute before the
	 * trailing edge of WS1/WS3 (which turns off the signals), thus
	 * causing the signals to be inactive during the DAC write.
	 */
	debi_write(s626ptr, LP_DACPOL, devpriv->Dacpol);

	/* TRANSFER OUTPUT DWORD VALUE INTO A2'S OUTPUT FIFO ---------------- */

	/* Copy DAC setpoint value to DAC's output DMA buffer. */

	/* WR7146( (uint32_t)devpriv->pDacWBuf, val ); */
	*devpriv->pDacWBuf = val;

	/* enab the output DMA transfer.  This will cause the DMAC to copy
	 * the DAC's data value to A2's output FIFO.  The DMA transfer will
	 * then immediately terminate because the protection address is
	 * reached upon transfer of the first DWORD value.
	 */
	MC_ENABLE(P_MC1, MC1_A2OUT);

	/*  While the DMA transfer is executing ... */

	/* Reset Audio2 output FIFO's underflow flag (along with any other
	 * FIFO underflow/overflow flags).  When set, this flag will
	 * indicate that we have emerged from slot 0.
	 */
	WR7146(P_ISR, ISR_AFOU);

	/* Wait for the DMA transfer to finish so that there will be data
	 * available in the FIFO when time slot 1 tries to transfer a DWORD
	 * from the FIFO to the output buffer register.  We test for DMA
	 * Done by polling the DMAC enable flag; this flag is automatically
	 * cleared when the transfer has finished.
	 */
	while ((RR7146(P_MC1) & MC1_A2OUT) != 0)
		rtdm_task_sleep(2000);

	/* START THE OUTPUT STREAM TO THE TARGET DAC -------------------- */

	/* FIFO data is now available, so we enable execution of time slots
	 * 1 and higher by clearing the EOS flag in slot 0.  Note that SD3
	 * will be shifted in and stored in FB_BUFFER2 for end-of-slot-list
	 * detection.
	 */
	SETVECT(0, XSD2 | RSD3 | SIB_A2);

	/* Wait for slot 1 to execute to ensure that the Packet will be
	 * transmitted.  This is detected by polling the Audio2 output FIFO
	 * underflow flag, which will be set when slot 1 execution has
	 * finished transferring the DAC's data DWORD from the output FIFO
	 * to the output buffer register.
	 */
	while ((RR7146(P_SSR) & SSR_AF2_OUT) == 0)
		rtdm_task_sleep(2000);

	/* Set up to trap execution at slot 0 when the TSL sequencer cycles
	 * back to slot 0 after executing the EOS in slot 5.  Also,
	 * simultaneously shift out and in the 0x00 that is ALWAYS the value
	 * stored in the last byte to be shifted out of the FIFO's DWORD
	 * buffer register.
	 */
	SETVECT(0, XSD2 | XFIFO_2 | RSD2 | SIB_A2 | EOS);

	/* WAIT FOR THE TRANSACTION TO FINISH ----------------------- */

	/* Wait for the TSL to finish executing all time slots before
	 * exiting this function.  We must do this so that the next DAC
	 * write doesn't start, thereby enabling clock/chip select signals:
	 *
	 * 1. Before the TSL sequence cycles back to slot 0, which disables
	 *    the clock/cs signal gating and traps slot // list execution.
	 *    we have not yet finished slot 5 then the clock/cs signals are
	 *    still gated and we have not finished transmitting the stream.
	 *
	 * 2. While slots 2-5 are executing due to a late slot 0 trap.  In
	 *    this case, the slot sequence is currently repeating, but with
	 *    clock/cs signals disabled.  We must wait for slot 0 to trap
	 *    execution before setting up the next DAC setpoint DMA transfer
	 *    and enabling the clock/cs signals.  To detect the end of slot 5,
	 *    we test for the FB_BUFFER2 MSB contents to be equal to 0xFF.  If
	 *    the TSL has not yet finished executing slot 5 ...
	 */
	if ((RR7146(P_FB_BUFFER2) & 0xFF000000) != 0) {
		/* The trap was set on time and we are still executing somewhere
		 * in slots 2-5, so we now wait for slot 0 to execute and trap
		 * TSL execution.  This is detected when FB_BUFFER2 MSB changes
		 * from 0xFF to 0x00, which slot 0 causes to happen by shifting
		 * out/in on SD2 the 0x00 that is always referenced by slot 5.
		 */
		while ((RR7146(P_FB_BUFFER2) & 0xFF000000) != 0)
			rtdm_task_sleep(2000);
	}
	/* Either (1) we were too late setting the slot 0 trap; the TSL
	 * sequencer restarted slot 0 before we could set the EOS trap flag,
	 * or (2) we were not late and execution is now trapped at slot 0.
	 * In either case, we must now change slot 0 so that it will store
	 * value 0xFF (instead of 0x00) to FB_BUFFER2 next time it executes.
	 * In order to do this, we reprogram slot 0 so that it will shift in
	 * SD3, which is driven only by a pull-up resistor.
	 */
	SETVECT(0, RSD3 | SIB_A2 | EOS);

	/* Wait for slot 0 to execute, at which time the TSL is setup for
	 * the next DAC write.  This is detected when FB_BUFFER2 MSB changes
	 * from 0x00 to 0xFF.
	 */
	while ((RR7146(P_FB_BUFFER2) & 0xFF000000) == 0)
		rtdm_task_sleep(2000);
}

/*  Private helper function: Write setpoint to an application DAC channel. */
static void set_adc(struct s626_struct * s626ptr, uint16_t chan, short dacdata)
{
	struct s626_priv *devpriv = s626ptr->private;
	register uint16_t signmask;
	register uint32_t WSImage;

	/*  Adjust DAC data polarity and set up Polarity Control Register */
	/*  image. */
	signmask = 1 << chan;
	if (dacdata < 0) {
		dacdata = -dacdata;
		devpriv->Dacpol |= signmask;
	}
	else
		devpriv->Dacpol	&= ~signmask;

	/*  Limit DAC setpoint value to valid range. */
	if ((uint16_t) dacdata > 0x1FFF)
		dacdata = 0x1FFF;

	/* Set up TSL2 records (aka "vectors") for DAC update.  Vectors V2
	 * and V3 transmit the setpoint to the target DAC.  V4 and V5 send
	 * data to a non-existent TrimDac channel just to keep the clock
	 * running after sending data to the target DAC.  This is necessary
	 * to eliminate the clock glitch that would otherwise occur at the
	 * end of the target DAC's serial data stream.  When the sequence
	 * restarts at V0 (after executing V5), the gate array automatically
	 * disables gating for the DAC clock and all DAC chip selects.
	 */

	WSImage = (chan & 2) ? WS1 : WS2;
	/* Choose DAC chip select to be asserted. */
	SETVECT(2, XSD2 | XFIFO_1 | WSImage);
	/* Slot 2: Transmit high data byte to target DAC. */
	SETVECT(3, XSD2 | XFIFO_0 | WSImage);
	/* Slot 3: Transmit low data byte to target DAC. */
	SETVECT(4, XSD2 | XFIFO_3 | WS3);
	/* Slot 4: Transmit to non-existent TrimDac channel to keep clock */
	SETVECT(5, XSD2 | XFIFO_2 | WS3 | EOS);
	/* Slot 5: running after writing target DAC's low data byte. */

	/*  Construct and transmit target DAC's serial packet:
	 * ( A10D DDDD ),( DDDD DDDD ),( 0x0F ),( 0x00 ) where A is chan<0>,
	 * and D<12:0> is the DAC setpoint.  Append a WORD value (that writes
	 * to a  non-existent TrimDac channel) that serves to keep the clock
	 * running after the packet has been sent to the target DAC.
	 */
	send_adc(s626ptr, 0x0F000000
		 /* Continue clock after target DAC data (write to non-existent trimdac). */
		 | 0x00004000
		 /* Address the two main dual-DAC devices (TSL's chip select enables
		  * target device). */
		 | ((uint32_t) (chan & 1) << 15)
		 /*  Address the DAC channel within the  device. */
		 | (uint32_t) dacdata);	/*  Include DAC setpoint data. */

}

static void write_trim_dac(struct s626_struct * s626ptr, uint8_t LogicalChan,
			   uint8_t DacData)
{

	struct s626_priv *devpriv = s626ptr->private;
	uint32_t chan;

	/*  Save the new setpoint in case the application needs to read it back later. */
	devpriv->TrimSetpoint[LogicalChan] = (uint8_t) DacData;

	/*  Map logical channel number to physical channel number. */
	chan = (uint32_t) trimchan[LogicalChan];

	/* Set up TSL2 records for TrimDac write operation.  All slots shift
	 * 0xFF in from pulled-up SD3 so that the end of the slot sequence
	 * can be detected.
	 */

	SETVECT(2, XSD2 | XFIFO_1 | WS3);
	/* Slot 2: Send high uint8_t to target TrimDac. */
	SETVECT(3, XSD2 | XFIFO_0 | WS3);
	/* Slot 3: Send low uint8_t to target TrimDac. */
	SETVECT(4, XSD2 | XFIFO_3 | WS1);
	/* Slot 4: Send NOP high uint8_t to DAC0 to keep clock running. */
	SETVECT(5, XSD2 | XFIFO_2 | WS1 | EOS);
	/* Slot 5: Send NOP low  uint8_t to DAC0. */

	/* Construct and transmit target DAC's serial packet:
	 * ( 0000 AAAA ), ( DDDD DDDD ),( 0x00 ),( 0x00 ) where A<3:0> is the
	 * DAC channel's address, and D<7:0> is the DAC setpoint.  Append a
	 * WORD value (that writes a channel 0 NOP command to a non-existent
	 * main DAC channel) that serves to keep the clock running after the
	 * packet has been sent to the target DAC.
	 */

	/*  Address the DAC channel within the trimdac device. */
	/*  Include DAC setpoint data. */
	send_adc(s626ptr, ((uint32_t) chan << 8) | (uint32_t) DacData);
}

static void load_trim_dacs(struct s626_struct * s626ptr)
{
	register uint8_t i;

	/*  Copy TrimDac setpoint values from EEPROM to TrimDacs. */
	for (i = 0; i < ARRAY_SIZE(trimchan); i++)
		write_trim_dac(s626ptr, i, i2c_read(s626ptr, trimadrs[i]));
}

/* ******  COUNTER FUNCTIONS  ******* */
/* All counter functions address a specific counter by means of the
 * "Counter" argument, which is a logical counter number.  The Counter
 * argument may have any of the following legal values: 0=0A, 1=1A,
 * 2=2A, 3=0B, 4=1B, 5=2B.
 */

/*  Read a counter's output latch. */
static uint32_t read_latch(struct s626_struct * s626ptr, struct enc_private *k)
{
	register uint32_t value;

	/*  Latch counts and fetch LSW of latched counts value. */
	value = (uint32_t) debi_read(s626ptr, k->my_latch_lsw);

	/*  Fetch MSW of latched counts and combine with LSW. */
	value |= ((uint32_t) debi_read(s626ptr, k->my_latch_lsw + 2) << 16);

	/*  Return latched counts. */
	return value;
}

/* Return/set a counter pair's latch trigger source.  0: On read
 * access, 1: A index latches A, 2: B index latches B, 3: A overflow
 * latches B.
 */
static void set_latch_source(struct s626_struct * s626ptr, struct enc_private *k,
			     uint16_t value)
{
	debi_replace(s626ptr, k->my_crb,
		     (uint16_t) (~(CRBMSK_INTCTRL | CRBMSK_LATCHSRC)),
		     (uint16_t) (value << CRBBIT_LATCHSRC));
}

static void s626_timer_load(struct s626_struct * s626ptr, struct enc_private *k,
			    int tick)
{
	uint16_t Setup = (LOADSRC_INDX << BF_LOADSRC) |	/*  preload upon */
			 /*  index. */
			 (INDXSRC_SOFT << BF_INDXSRC) |	 /*  Disable hardware index. */
			 (CLKSRC_TIMER << BF_CLKSRC) |	 /*  Operating mode is Timer. */
			 (CLKPOL_POS << BF_CLKPOL) |	 /*  Active high clock. */
			 (CNTDIR_DOWN << BF_CLKPOL) |	 /*  Count direction is Down. */
			 (CLKMULT_1X << BF_CLKMULT) |	 /*  Clock multiplier is 1x. */
			 (CLKENAB_INDEX << BF_CLKENAB);

	uint16_t valueSrclatch = LATCHSRC_A_INDXA;

	k->set_mode(s626ptr, k, Setup, FALSE);

	/*  Set the preload register */
	preload(s626ptr, k, tick);

	/*  Software index pulse forces the preload register to load */
	/*  into the counter */
	k->set_load_trig(s626ptr, k, 0);
	k->pulse_index(s626ptr, k);

	/* set reload on counter overflow */
	k->set_load_trig(s626ptr, k, 1);

	/* set interrupt on overflow */
	k->set_int_src(s626ptr, k, INTSRC_OVER);

	set_latch_source(s626ptr, k, valueSrclatch);
}

int s626_irq_handler(unsigned int irq, void *d)
{
	struct a4l_device *dev = NULL;
	struct s626_priv *devpriv = NULL;

	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (a4l_get_irq(s626ptr->dev) == irq) {
			dev = s626ptr->dev;
			devpriv = s626ptr->private;
			break;
		}
	}

	if (dev == NULL) {
		__a4l_err( "Inside %s, error, no device\n", __PRETTY_FUNCTION__);
		return -ENODEV;
	}

	rtdm_lock_get_irqsave(&dev->lock, s626ptr->flags);

	/* save interrupt enable register state */
	s626ptr->irqstatus = readl(devpriv->base_addr + P_IER);

	/* read interrupt type */
	s626ptr->irqtype = readl(devpriv->base_addr + P_ISR);

	/* disable master interrupt */
	writel(0, devpriv->base_addr + P_IER);

	/* clear interrupt */
	writel(s626ptr->irqtype, devpriv->base_addr + P_ISR);

	rtdm_lock_put_irqrestore(&dev->lock, s626ptr->flags);

	/* let the s626_irq_service deal with the irq */
	rtdm_event_pulse(&devpriv->synch);

	return IRQ_HANDLED;
}


static inline void s626_irq_handle_end_of_scan(struct s626_struct *s626ptr)
{
	struct s626_priv *devpriv = s626ptr->private;
	struct a4l_device *dev = s626ptr->dev;

	int32_t *readaddr;
	a4l_cmd_t *cmd;
	struct a4l_subdevice *s;
	short tempdata;
	int i;

	s = a4l_get_subd(dev, SUBD_AI);
	cmd = a4l_get_cmd(s);

	/* Init ptr to DMA buffer that holds new ADC data.  We skip the
	  first uint16_t in the buffer because it contains junk data from
	  the final ADC of the previous poll list scan. */

	readaddr = (int32_t *) devpriv->ANABuf.LogicalBase + 1;

	/*  get the data and hand it over */
	for (i = 0; i < (cmd->nb_chan); i++) {
		/* Convert ADC data to 16-bit integer values and copy to application
		  buffer */
		tempdata = s626_ai_reg_to_uint((int) *readaddr);
		readaddr++;
	}

	if (!(devpriv->ai_continous))
		devpriv->ai_sample_count--;

	if (devpriv->ai_sample_count <= 0) {
		devpriv->ai_cmd_running = 0;

		/*  Stop RPS program. */
		MC_DISABLE(P_MC1, MC1_ERPS1);

		/* disable master interrupt */
		s626ptr->irqstatus = 0;
	}

	if (devpriv->ai_cmd_running && cmd->scan_begin_src == TRIG_EXT)
		s626_dio_set_irq(s626ptr, cmd->scan_begin_arg);
}

static inline void s626_irq_handle_dio_and_counter(struct s626_struct *s626ptr)
{
	struct s626_priv *devpriv = s626ptr->private;
	struct a4l_device *dev = s626ptr->dev;

	struct s626_subd_dio_private *sdio_priv;
	struct enc_private *k;
	struct a4l_subdevice *s, *t;
	a4l_cmd_t *cmd;

	uint8_t group;
	uint16_t irqbit;
	uint16_t shift;

	s = a4l_get_subd(dev, SUBD_AI);
	cmd = a4l_get_cmd(s);

	for (group = 0; group < S626_DIO_BANKS; group++) {
		irqbit = 0;

		t = a4l_get_subd(s626ptr->dev, group + SUBD_DIO_A);
		sdio_priv = (struct s626_subd_dio_private *) &t->priv;

		/* read interrupt type */
		irqbit = debi_read(s626ptr, sdio_priv->rd_cap_flg);

		/* check if interrupt is generated from dio channels */
		if (!irqbit)
			continue;

		s626_dio_reset_irq(s626ptr, group, irqbit);
		if (!devpriv->ai_cmd_running)
			break;

		/* check if interrupt is an ai acquisition start trigger */
		shift = cmd->start_arg - 16 * group;
		if (irqbit >> shift == 1 && cmd->start_src == TRIG_EXT) {
			/*  Start executing the RPS program. */
			MC_ENABLE(P_MC1, MC1_ERPS1);
			if (cmd->scan_begin_src == TRIG_EXT)
				s626_dio_set_irq(s626ptr,cmd->scan_begin_arg);
		}

		shift = cmd->scan_begin_arg - 16 * group;
		if (irqbit >> shift == 1 && cmd->scan_begin_src == TRIG_EXT) {
			/*  Trigger ADC scan loop start by setting RPS Signal 0. */
			MC_ENABLE(P_MC2, MC2_ADC_RPS);
			if (cmd->convert_src == TRIG_EXT) {
				devpriv->ai_convert_count = cmd->nb_chan;
				s626_dio_set_irq(s626ptr, cmd->convert_arg);
			}
			if (cmd->convert_src == TRIG_TIMER) {
				k = &encpriv[5];
				devpriv->ai_convert_count = cmd->nb_chan;
				k->set_enable(s626ptr,k, CLKENAB_ALWAYS);
			}
		}

		shift = cmd->convert_arg - 16 * group;
		if (irqbit >> shift == 1 && cmd->convert_src == TRIG_EXT) {
			/*  Trigger ADC scan loop start by setting RPS Signal 0. */
			MC_ENABLE(P_MC2, MC2_ADC_RPS);
			devpriv->ai_convert_count--;
			if (devpriv->ai_convert_count > 0)
				s626_dio_set_irq(s626ptr,cmd->convert_arg);
		}
		break;
	}

	/* read interrupt type */
	irqbit = debi_read(s626ptr, LP_RDMISC2);

	/* check interrupt on counters */
	if (irqbit & IRQ_COINT1A) {
		k = &encpriv[0];
		/* clear interrupt capture flag */
		k->reset_cap_flags(s626ptr, k);
	}
	if (irqbit & IRQ_COINT2A) {
		k = &encpriv[1];
		k->reset_cap_flags(s626ptr, k);
	}
	if (irqbit & IRQ_COINT3A) {
		k = &encpriv[2];
		k->reset_cap_flags(s626ptr, k);
	}
	if (irqbit & IRQ_COINT1B) {
		k = &encpriv[3];
		k->reset_cap_flags(s626ptr, k);
	}
	if (irqbit & IRQ_COINT2B) {
		k = &encpriv[4];
		k->reset_cap_flags(s626ptr, k);
		if (devpriv->ai_convert_count > 0) {
			devpriv->ai_convert_count--;
			if (devpriv->ai_convert_count == 0)
				k->set_enable(s626ptr, k, CLKENAB_INDEX);

			if (cmd->convert_src == TRIG_TIMER) {
				/* Trigger ADC scan loop start by setting RPS
				   Signal 0. */
				MC_ENABLE(P_MC2, MC2_ADC_RPS);
			}
		}
	}
	if (irqbit & IRQ_COINT3B) {
		k = &encpriv[5];
		k->reset_cap_flags(s626ptr, k);

		if (cmd->scan_begin_src == TRIG_TIMER) {
			MC_ENABLE(P_MC2, MC2_ADC_RPS);
		}

		if (cmd->convert_src == TRIG_TIMER) {
			k = &encpriv[4];
			devpriv->ai_convert_count = cmd->nb_chan;
			k->set_enable(s626ptr, k, CLKENAB_ALWAYS);
		}
	}
}

void s626_irq_service(void *arg)
{
	struct s626_struct *s626ptr = arg;
	struct s626_priv *devpriv = s626ptr->private;
	int ret;

	for (;!s626ptr->terminate_task;) {

		ret = rtdm_event_wait(&devpriv->synch);
		if (ret)
			break;

		switch (s626ptr->irqtype) {
		case IRQ_RPS1:
			s626_irq_handle_end_of_scan(s626ptr);
			break;
		case IRQ_GPIO3:
			s626_irq_handle_dio_and_counter(s626ptr);
			break;
		}

		/* enable interrupt */
		writel(s626ptr->irqstatus, devpriv->base_addr + P_IER);
	}
}

/* This function doesn't require a particular form, this is just what
 * happens to be used in some of the drivers.  It should convert ns
 * nanoseconds to a counter value suitable for programming the device.
 * Also, it should adjust ns so that it cooresponds to the actual time
 * that the device will use. */
static int s626_ns_to_timer(int *nanosec, int round_mode)
{
	int divider, base;

	/* 2MHz internal clock */
	base = 500;

	switch (round_mode) {
	case TRIG_ROUND_NEAREST:
	default:
		divider = (*nanosec + base / 2) / base;
		break;
	case TRIG_ROUND_DOWN:
		divider = (*nanosec) / base;
		break;
	case TRIG_ROUND_UP:
		divider = (*nanosec + base - 1) / base;
		break;
	}

	*nanosec = base * divider;
	return divider - 1;
}

static int s626_allocate_dma_buffers(struct s626_struct * s626ptr)
{
	struct pci_dev *pcidev = s626ptr->pcidev;
	struct s626_priv *devpriv = s626ptr->private;
	void *addr;
	dma_addr_t appdma;

	addr = pci_alloc_consistent(pcidev, DMABUF_SIZE, &appdma);
	if (!addr)
		return -ENOMEM;
	devpriv->ANABuf.LogicalBase = addr;
	devpriv->ANABuf.PhysicalBase = appdma;

	addr = pci_alloc_consistent(pcidev, DMABUF_SIZE, &appdma);
	if (!addr)
		return -ENOMEM;
	devpriv->RPSBuf.LogicalBase = addr;
	devpriv->RPSBuf.PhysicalBase = appdma;

	return 0;
}

static int s626_dio_clear_irq(struct s626_struct * s626ptr)
{
	struct s626_subd_dio_private *sdio_priv;
	unsigned int group;
	struct a4l_subdevice *t;

	/* disable edge capture write command */
	debi_write(s626ptr, LP_MISC1, MISC1_NOEDCAP);

	for (group = 0; group < S626_DIO_BANKS; group++) {

		t = a4l_get_subd(s626ptr->dev, group + SUBD_DIO_A);
		sdio_priv = (struct s626_subd_dio_private *) &t->priv;

		/* clear pending events and interrupt */
		debi_write(s626ptr, sdio_priv->wr_cap_sel, 0xffff);
	}

	return 0;
}

static int s626_dio_set_irq(struct s626_struct * s626ptr, unsigned int chan)
{
	struct s626_subd_dio_private *sdio_priv;
	unsigned int bitmask;
	unsigned int status;
	unsigned int group;
	struct a4l_subdevice *t;

	/* select dio bank */
	group = chan / 16;
	bitmask = 1 << (chan - (16 * group));

	t = a4l_get_subd(s626ptr->dev, group + SUBD_DIO_A);
	sdio_priv = (struct s626_subd_dio_private *) &t->priv;

	/* set channel to capture positive edge */
	status = debi_read(s626ptr, sdio_priv->rd_edg_sel);
	debi_write(s626ptr, sdio_priv->wr_edg_sel, bitmask | status);

	/* enable interrupt on selected channel */
	status = debi_read(s626ptr, sdio_priv->rd_int_sel);

	debi_write(s626ptr, sdio_priv->wr_int_sel, bitmask | status);

	/* enable edge capture write command */
	debi_write(s626ptr, LP_MISC1, MISC1_EDCAP);

	/* enable edge capture on selected channel */
	status = debi_read(s626ptr, sdio_priv->rd_cap_sel);

	debi_write(s626ptr, sdio_priv->wr_cap_sel, bitmask | status);

	return 0;
}

static int s626_dio_reset_irq(struct s626_struct * s626ptr, unsigned int group,
			      unsigned int mask)
{
	struct s626_subd_dio_private *sdio_priv;
	struct a4l_subdevice *t;

	t = a4l_get_subd(s626ptr->dev, group + SUBD_DIO_A);
	sdio_priv = (struct s626_subd_dio_private *) &t->priv;

	/* disable edge capture write command */
	debi_write(s626ptr, LP_MISC1, MISC1_NOEDCAP);

	/* enable edge capture on selected channel */
	debi_write(s626ptr, sdio_priv->wr_cap_sel, mask);

	return 0;
}

/*
 * this functions build the RPS program for hardware driven acquistion
 */
static void ResetADC(struct s626_struct * s626ptr, uint8_t *ppl, a4l_cmd_t * cmd)
{
	struct s626_priv *devpriv = s626ptr->private;
	register uint32_t *pRPS;
	uint32_t JmpAdrs;
	uint16_t i;
	uint16_t n;
	uint32_t LocalPPL;

	/*  Stop RPS program in case it is currently running. */
	MC_DISABLE(P_MC1, MC1_ERPS1);

	/*  Set starting logical address to write RPS commands. */
	pRPS = (uint32_t *) devpriv->RPSBuf.LogicalBase;

	/*  Initialize RPS instruction pointer. */
	WR7146(P_RPSADDR1, (uint32_t) devpriv->RPSBuf.PhysicalBase);

	/*  Construct RPS program in RPSBuf DMA buffer */

	if (cmd != NULL && cmd->scan_begin_src != TRIG_FOLLOW) {
		/*  Wait for Start trigger. */
		*pRPS++ = RPS_PAUSE | RPS_SIGADC;
		*pRPS++ = RPS_CLRSIGNAL | RPS_SIGADC;
	}

	/* SAA7146 BUG WORKAROUND Do a dummy DEBI Write.  This is necessary
	 * because the first RPS DEBI Write following a non-RPS DEBI write
	 * seems to always fail.  If we don't do this dummy write, the ADC
	 * gain might not be set to the value required for the first slot in
	 * the poll list; the ADC gain would instead remain unchanged from
	 * the previously programmed value.
	 */
	*pRPS++ = RPS_LDREG | (P_DEBICMD >> 2);
	/* Write DEBI Write command and address to shadow RAM. */

	*pRPS++ = DEBI_CMD_WRWORD | LP_GSEL;
	*pRPS++ = RPS_LDREG | (P_DEBIAD >> 2);
	/*  Write DEBI immediate data  to shadow RAM: */

	*pRPS++ = GSEL_BIPOLAR5V;
	/*  arbitrary immediate data  value. */

	*pRPS++ = RPS_CLRSIGNAL | RPS_DEBI;
	/*  Reset "shadow RAM  uploaded" flag. */
	*pRPS++ = RPS_UPLOAD | RPS_DEBI; /*  Invoke shadow RAM upload. */
	*pRPS++ = RPS_PAUSE | RPS_DEBI;	/*  Wait for shadow upload to finish. */

	/* Digitize all slots in the poll list. This is implemented as a
	 * for loop to limit the slot count to 16 in case the application
	 * forgot to set the EOPL flag in the final slot.
	 */
	for (devpriv->AdcItems = 0; devpriv->AdcItems < 16; devpriv->AdcItems++) {
		/* Convert application's poll list item to private board class
		 * format.  Each app poll list item is an uint8_t with form
		 * (EOPL,x,x,RANGE,CHAN<3:0>), where RANGE code indicates 0 =
		 * +-10V, 1 = +-5V, and EOPL = End of Poll List marker.
		 */
		LocalPPL = (*ppl << 8) | (*ppl & 0x10 ? GSEL_BIPOLAR5V : GSEL_BIPOLAR10V);

		/*  Switch ADC analog gain. */
		*pRPS++ = RPS_LDREG | (P_DEBICMD >> 2);	/*  Write DEBI command */
		/*  and address to */
		/*  shadow RAM. */
		*pRPS++ = DEBI_CMD_WRWORD | LP_GSEL;
		*pRPS++ = RPS_LDREG | (P_DEBIAD >> 2); /*  Write DEBI */
		/*  immediate data to */
		/*  shadow RAM. */
		*pRPS++ = LocalPPL;
		*pRPS++ = RPS_CLRSIGNAL | RPS_DEBI; /*  Reset "shadow RAM uploaded" */
		/*  flag. */
		*pRPS++ = RPS_UPLOAD | RPS_DEBI; /*  Invoke shadow RAM upload. */
		*pRPS++ = RPS_PAUSE | RPS_DEBI;	/*  Wait for shadow upload to */
		/*  finish. */

		/*  Select ADC analog input channel. */
		*pRPS++ = RPS_LDREG | (P_DEBICMD >> 2);
		/*  Write DEBI command and address to  shadow RAM. */
		*pRPS++ = DEBI_CMD_WRWORD | LP_ISEL;
		*pRPS++ = RPS_LDREG | (P_DEBIAD >> 2);
		/*  Write DEBI immediate data to shadow RAM. */
		*pRPS++ = LocalPPL;
		*pRPS++ = RPS_CLRSIGNAL | RPS_DEBI;
		/*  Reset "shadow RAM uploaded"  flag. */

		*pRPS++ = RPS_UPLOAD | RPS_DEBI;
		/*  Invoke shadow RAM upload. */

		*pRPS++ = RPS_PAUSE | RPS_DEBI;
		/*  Wait for shadow upload to finish. */

		/* Delay at least 10 microseconds for analog input settling.
		 * Instead of padding with NOPs, we use RPS_JUMP instructions
		 * here; this allows us to produce a longer delay than is
		 * possible with NOPs because each RPS_JUMP flushes the RPS'
		 * instruction prefetch pipeline.
		 */
		JmpAdrs = (uint32_t) devpriv->RPSBuf.PhysicalBase
			  + (uint32_t) ((unsigned long) pRPS
					- (unsigned long) devpriv->RPSBuf.LogicalBase);
		for (i = 0; i < (10 * RPSCLK_PER_US / 2); i++) {
			JmpAdrs += 8; /*  Repeat to implement time delay: */
			*pRPS++ = RPS_JUMP; /*  Jump to next RPS instruction. */
			*pRPS++ = JmpAdrs;
		}

		if (cmd != NULL && cmd->convert_src != TRIG_NOW) {
			/*  Wait for Start trigger. */
			*pRPS++ = RPS_PAUSE | RPS_SIGADC;
			*pRPS++ = RPS_CLRSIGNAL | RPS_SIGADC;
		}
		/*  Start ADC by pulsing GPIO1. */
		*pRPS++ = RPS_LDREG | (P_GPIO >> 2); /*  Begin ADC Start pulse. */
		*pRPS++ = GPIO_BASE | GPIO1_LO;
		*pRPS++ = RPS_NOP;
		/*  VERSION 2.03 CHANGE: STRETCH OUT ADC START PULSE. */
		*pRPS++ = RPS_LDREG | (P_GPIO >> 2); /*  End ADC Start pulse. */
		*pRPS++ = GPIO_BASE | GPIO1_HI;

		/* Wait for ADC to complete (GPIO2 is asserted high when ADC not
		 * busy) and for data from previous conversion to shift into FB
		 * BUFFER 1 register.
		 */
		*pRPS++ = RPS_PAUSE | RPS_GPIO2; /*  Wait for ADC done. */

		/*  Transfer ADC data from FB BUFFER 1 register to DMA buffer. */
		*pRPS++ = RPS_STREG | (BUGFIX_STREG(P_FB_BUFFER1) >> 2);
		*pRPS++ = (uint32_t) devpriv->ANABuf.PhysicalBase
			  + (devpriv->AdcItems << 2);

		/*  If this slot's EndOfPollList flag is set, all channels have */
		/*  now been processed. */
		if (*ppl++ & EOPL) {
			devpriv->AdcItems++; /*  Adjust poll list item count. */
			break; /*  Exit poll list processing loop. */
		}
	}

	/* VERSION 2.01 CHANGE: DELAY CHANGED FROM 250NS to 2US.  Allow the
	 * ADC to stabilize for 2 microseconds before starting the final
	 * (dummy) conversion.  This delay is necessary to allow sufficient
	 * time between last conversion finished and the start of the dummy
	 * conversion.  Without this delay, the last conversion's data value
	 * is sometimes set to the previous conversion's data value.
	 */
	for (n = 0; n < (2 * RPSCLK_PER_US); n++)
		*pRPS++ = RPS_NOP;

	/* Start a dummy conversion to cause the data from the last
	 * conversion of interest to be shifted in.
	 */
	*pRPS++ = RPS_LDREG | (P_GPIO >> 2); /*  Begin ADC Start pulse. */
	*pRPS++ = GPIO_BASE | GPIO1_LO;
	*pRPS++ = RPS_NOP;
	/* VERSION 2.03 CHANGE: STRETCH OUT ADC START PULSE. */
	*pRPS++ = RPS_LDREG | (P_GPIO >> 2); /*  End ADC Start pulse. */
	*pRPS++ = GPIO_BASE | GPIO1_HI;

	/* Wait for the data from the last conversion of interest to arrive
	 * in FB BUFFER 1 register.
	 */
	*pRPS++ = RPS_PAUSE | RPS_GPIO2; /*  Wait for ADC done. */

	/*  Transfer final ADC data from FB BUFFER 1 register to DMA buffer. */
	*pRPS++ = RPS_STREG | (BUGFIX_STREG(P_FB_BUFFER1) >> 2); /*  */
	*pRPS++ = (uint32_t) devpriv->ANABuf.PhysicalBase + (devpriv->AdcItems << 2);

	/*  Indicate ADC scan loop is finished. */
	/*  *pRPS++= RPS_CLRSIGNAL | RPS_SIGADC ;  // Signal ReadADC() that scan is done. */

	/* invoke interrupt */
	if (devpriv->ai_cmd_running == 1) {
		*pRPS++ = RPS_IRQ;
	}
	/*  Restart RPS program at its beginning. */
	*pRPS++ = RPS_JUMP; /*  Branch to start of RPS program. */
	*pRPS++ = (uint32_t) devpriv->RPSBuf.PhysicalBase;

	/*  End of RPS program build */
}

/* TO COMPLETE, IF NECESSARY */
static int s626_ai_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	return -EINVAL;
}

static uint16_t s626_ai_reg_to_uint(int data)
{
	unsigned int tempdata;

	tempdata = (data >> 18);
	if (tempdata & 0x2000)
		tempdata &= 0x1fff;
	else
		tempdata += (1 << 13);

	return(tempdata & 0xFFFF);
}

static int s626_ai_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	uint16_t chan = CR_CHAN(insn->chan_desc);
	uint16_t range = CR_RNG(insn->chan_desc);
	uint16_t AdcSpec = 0;
	uint16_t *data = (uint16_t *) insn->data;
	uint32_t GpioImage;
	int n;

	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (devpriv == NULL) {
		__a4l_err( "Inside %s, error, no device\n", __PRETTY_FUNCTION__);
		return -ENODEV;
	}

	/* Convert application's ADC specification into form
	 *  appropriate for register programming.
	 */
	if (range == 0)
		AdcSpec = (chan << 8) | (GSEL_BIPOLAR5V);
	else
		AdcSpec	= (chan << 8) | (GSEL_BIPOLAR10V);

	/*  Switch ADC analog gain. */
	debi_write(s626ptr, LP_GSEL, AdcSpec);

	/*  Select ADC analog input channel. */
	debi_write(s626ptr, LP_ISEL, AdcSpec);

	for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {

		/*  Delay 10 microseconds for analog input settling. */
		udelay(10);

		/*  Start ADC by pulsing GPIO1 low. */
		GpioImage = RR7146(P_GPIO);
		/*  Assert ADC Start command */
		WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
		/*    and stretch it out. */
		WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
		WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
		/*  Negate ADC Start command. */
		WR7146(P_GPIO, GpioImage | GPIO1_HI);

		/*  Wait for ADC to complete (GPIO2 is asserted high when */
		/*  ADC not busy) and for data from previous conversion to */
		/*  shift into FB BUFFER 1 register. */

		/*  Wait for ADC done. */
		while (!(RR7146(P_PSR) & PSR_GPIO2))
			rtdm_task_sleep(2000);

		/*  Fetch ADC data. */
		if (n != 0)
			data[n - 1] = s626_ai_reg_to_uint(RR7146(P_FB_BUFFER1));

		/* Allow the ADC to stabilize for 4 microseconds before
		 * starting the next (final) conversion.  This delay is
		 * necessary to allow sufficient time between last
		 * conversion finished and the start of the next
		 * conversion.  Without this delay, the last conversion's
		 * data value is sometimes set to the previous
		 * conversion's data value.
		 */
		udelay(4);
	}

	/* Start a dummy conversion to cause the data from the
	 * previous conversion to be shifted in. */
	GpioImage = RR7146(P_GPIO);

	/* Assert ADC Start command */
	WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
	/*    and stretch it out. */
	WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
	WR7146(P_GPIO, GpioImage & ~GPIO1_HI);
	/*  Negate ADC Start command. */
	WR7146(P_GPIO, GpioImage | GPIO1_HI);

	/*  Wait for the data to arrive in FB BUFFER 1 register. */

	/*  Wait for ADC done. */
	while (!(RR7146(P_PSR) & PSR_GPIO2))
		rtdm_task_sleep(2000);

	/*  Fetch ADC data from audio interface's input shift register. */

	/*  Fetch ADC data. */
	if (n != 0)
		data[n - 1] = s626_ai_reg_to_uint(RR7146(P_FB_BUFFER1));

	return n;
}

static int s626_ai_load_polllist(uint8_t *ppl, a4l_cmd_t *cmd)
{

	int n;

	for (n = 0; n < cmd->nb_chan; n++) {
		if (CR_RNG((cmd->chan_descs)[n]) == 0)
			ppl[n] = (CR_CHAN((cmd->chan_descs)[n])) | (RANGE_5V);
		else
			ppl[n] = (CR_CHAN((cmd->chan_descs)[n])) | (RANGE_10V);
	}
	if (n != 0)
		ppl[n - 1] |= EOPL;

	return n;
}

static int s626_ai_inttrig(struct a4l_subdevice * subdev,
			   unsigned long int trignum)
{
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subdev->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (devpriv == NULL) {
		__a4l_err( "Inside %s, error, no device\n", __PRETTY_FUNCTION__);
		return -ENODEV;
	}

	if (trignum != 0)
		return -EINVAL;

	/*  Start executing the RPS program. */
	MC_ENABLE(P_MC1, MC1_ERPS1);

	subdev->trigger = NULL;

	return 1;
}

/*  TO COMPLETE  */
static int s626_ai_cmd(struct a4l_subdevice * subdev, a4l_cmd_t * cmd)
{
	uint8_t ppl[16];
	struct enc_private *k;
	int tick;

	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subdev->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (devpriv == NULL) {
		__a4l_err( "Inside %s, error, no device\n", __PRETTY_FUNCTION__);
		return -ENODEV;
	}

	if (devpriv->ai_cmd_running) {
		printk(KERN_ERR "s626_ai_cmd: Another ai_cmd is running\n");
		return -EBUSY;
	}
	/* disable interrupt */
	writel(0, devpriv->base_addr + P_IER);

	/* clear interrupt request */
	writel(IRQ_RPS1 | IRQ_GPIO3, devpriv->base_addr + P_ISR);

	/* clear any pending interrupt */
	s626_dio_clear_irq(s626ptr);
	/*   s626_enc_clear_irq(dev); */

	/* reset ai_cmd_running flag */
	devpriv->ai_cmd_running = 0;

	/*  test if cmd is valid */
	if (cmd == NULL)
		return -EINVAL;

	if (a4l_get_irq(s626ptr->dev) == 0) {
		__a4l_err("s626_ai_cmd: cannot run command without an irq\n");
		return -EIO;
	}

	s626_ai_load_polllist(ppl, cmd);
	devpriv->ai_cmd_running = 1;
	devpriv->ai_convert_count = 0;

	switch (cmd->scan_begin_src) {
	case TRIG_FOLLOW:
		break;
	case TRIG_TIMER:
		/*  set a conter to generate adc trigger at scan_begin_arg interval */
		k = &encpriv[5];
		tick = s626_ns_to_timer((int *) &cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);

		/* load timer value and enable interrupt */
		s626_timer_load(s626ptr, k, tick);
		k->set_enable(s626ptr, k, CLKENAB_ALWAYS);
		break;
	case TRIG_EXT:
		/*  set the digital line and interrupt for scan trigger */
		if (cmd->start_src != TRIG_EXT)
			s626_dio_set_irq(s626ptr, cmd->scan_begin_arg);
		break;
	}

	switch (cmd->convert_src) {
	case TRIG_NOW:
		break;
	case TRIG_TIMER:
		/*  set a conter to generate adc trigger at convert_arg interval */
		k = &encpriv[4];
		tick = s626_ns_to_timer((int *) &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);

		/* load timer value and enable interrupt */
		s626_timer_load(s626ptr, k, tick);
		k->set_enable(s626ptr, k, CLKENAB_INDEX);
		break;
	case TRIG_EXT:
		/*  set the digital line and interrupt for convert trigger */
		if (cmd->scan_begin_src != TRIG_EXT && cmd->start_src == TRIG_EXT)
			s626_dio_set_irq(s626ptr, cmd->convert_arg);
		break;
	}

	switch (cmd->stop_src) {
	case TRIG_COUNT:
		/*  data arrives as one packet */
		devpriv->ai_sample_count = cmd->stop_arg;
		devpriv->ai_continous = 0;
		break;
	case TRIG_NONE:
		/*  continous acquisition */
		devpriv->ai_continous = 1;
		devpriv->ai_sample_count = 1;
		break;
	}

	ResetADC(s626ptr, ppl, cmd);

	switch (cmd->start_src) {
	case TRIG_NOW:
		/*  Trigger ADC scan loop start by setting RPS Signal 0. */
		/*  MC_ENABLE( P_MC2, MC2_ADC_RPS ); */

		/*  Start executing the RPS program. */
		MC_ENABLE(P_MC1, MC1_ERPS1);

		subdev->trigger = NULL;
		break;
	case TRIG_EXT:
		/* configure DIO channel for acquisition trigger */
		s626_dio_set_irq(s626ptr, cmd->start_arg);

		subdev->trigger = NULL;
		break;
	case TRIG_INT:
		subdev->trigger = s626_ai_inttrig;
		break;
	}

	/* enable interrupt */
	writel(IRQ_GPIO3 | IRQ_RPS1, devpriv->base_addr + P_IER);

	return 0;
}

#define TRIG_INVALID    0x00000000

static inline int cfc_check_trigger_src(unsigned int *src, unsigned int flags)
{
	unsigned int orig_src = *src;

	*src = orig_src & flags;
	if (*src == TRIG_INVALID || *src != orig_src)
		return -EINVAL;
	return 0;
}

static inline int cfc_check_trigger_is_unique(unsigned int src)
{
	/* this test is true if more than one _src bit is set */
	if ((src & (src - 1)) != 0)
		return -EINVAL;
	return 0;
}

static inline int cfc_check_trigger_arg_is(unsigned int *arg, unsigned int val)
{
	if (*arg != val) {
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

static inline int cfc_check_trigger_arg_min(unsigned int *arg, unsigned int val)
{
	if (*arg < val) {
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

static inline int cfc_check_trigger_arg_max(unsigned int *arg, unsigned int val)
{
	if (*arg > val) {
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

static int s626_ai_cmdtest(struct a4l_subdevice * subdev, a4l_cmd_t * cmd)
{
	int err = 0;
	int tmp;

	/* Step 1 : check if triggers are trivially valid */

	err |= cfc_check_trigger_src(&cmd->start_src, TRIG_NOW | TRIG_INT | TRIG_EXT);
	err |= cfc_check_trigger_src(&cmd->scan_begin_src,
				     TRIG_TIMER | TRIG_EXT | TRIG_FOLLOW);
	err |= cfc_check_trigger_src(&cmd->convert_src,
				     TRIG_TIMER | TRIG_EXT | TRIG_NOW);
	err |= cfc_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
	err |= cfc_check_trigger_src(&cmd->stop_src, TRIG_COUNT | TRIG_NONE);

	if (err)
		return 1;

	/* Step 2a : make sure trigger sources are unique */

	err |= cfc_check_trigger_is_unique(cmd->start_src);
	err |= cfc_check_trigger_is_unique(cmd->scan_begin_src);
	err |= cfc_check_trigger_is_unique(cmd->convert_src);
	err |= cfc_check_trigger_is_unique(cmd->stop_src);

	/* Step 2b : and mutually compatible */

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_src != TRIG_EXT)
		err |= cfc_check_trigger_arg_is(&cmd->start_arg, 0);
	if (cmd->start_src == TRIG_EXT)
		err |= cfc_check_trigger_arg_max(&cmd->start_arg, 39);

	if (cmd->scan_begin_src == TRIG_EXT)
		err |= cfc_check_trigger_arg_max(&cmd->scan_begin_arg, 39);

	if (cmd->convert_src == TRIG_EXT)
		err |= cfc_check_trigger_arg_max(&cmd->convert_arg, 39);

	if (cmd->scan_begin_src == TRIG_TIMER) {
		err |= cfc_check_trigger_arg_min(&cmd->scan_begin_arg, MAX_SPEED);
		err |= cfc_check_trigger_arg_max(&cmd->scan_begin_arg, MIN_SPEED);
	}
	else {
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		/*    err |= cfc_check_trigger_arg_max(&cmd->scan_begin_arg, 9); */
	}
	if (cmd->convert_src == TRIG_TIMER) {
		err |= cfc_check_trigger_arg_min(&cmd->convert_arg, MAX_SPEED);
		err |= cfc_check_trigger_arg_max(&cmd->convert_arg, MIN_SPEED);
	}
	else {
		/* external trigger */
		/* see above */
		/*    err |= cfc_check_trigger_arg_max(&cmd->scan_begin_arg, 9); */
	}

	err |= cfc_check_trigger_arg_is(&cmd->scan_end_arg, cmd->nb_chan);

	if (cmd->stop_src == TRIG_COUNT)
		err |= cfc_check_trigger_arg_max(&cmd->stop_arg, 0x00ffffff);
	else
		/* TRIG_NONE */
		err |= cfc_check_trigger_arg_is(&cmd->stop_arg, 0);

	if (err)
		return 3;

	/* step 4: fix up any arguments */
	if (cmd->scan_begin_src == TRIG_TIMER) {
		tmp = cmd->scan_begin_arg;
		s626_ns_to_timer((int *) &cmd->scan_begin_arg,
				 cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->scan_begin_arg)
			err++;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		s626_ns_to_timer((int *) &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			err++;
		if (cmd->scan_begin_src == TRIG_TIMER &&
		    cmd->scan_begin_arg < cmd->convert_arg * cmd->scan_end_arg) {
			cmd->scan_begin_arg = cmd->convert_arg * cmd->scan_end_arg;
			err++;
		}
	}

	if (err)
		return 4;

	return 0;
}

static void s626_ai_cancel(struct a4l_subdevice * subdev)
{
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subdev->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (devpriv == NULL)
		return;

	/*  Stop RPS program in case it is currently running. */
	MC_DISABLE(P_MC1, MC1_ERPS1);

	/* disable master interrupt */
	writel(0, devpriv->base_addr + P_IER);

	devpriv->ai_cmd_running = 0;
}

static int s626_ao_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	struct s626_subd_ao_private *subdpriv =
	(struct s626_subd_ao_private *) subd->priv;
	uint16_t *data = (uint16_t *) insn->data;
	int chan = CR_CHAN(insn->chan_desc);
	int16_t dacdata;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	dacdata = (int16_t) data[0];
	dacdata -= (0x1fff);

	subdpriv->readback[chan] = dacdata;

	set_adc(s626ptr, chan, dacdata);

	return 0;
}

static int s626_ao_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{

	uint16_t *data = (uint16_t *) insn->data;
	struct s626_subd_ao_private *subdpriv;
	int chan = CR_CHAN(insn->chan_desc);

	subdpriv = (struct s626_subd_ao_private *) subd->priv;
	data[0] = subdpriv->readback[chan];

	return 0;
}

/* *************** DIGITAL I/O FUNCTIONS ***************
 * All DIO functions address a group of DIO channels by means of
 * "group" argument.  group may be 0, 1 or 2, which correspond to DIO
 * ports A, B and C, respectively.
 */

static void s626_dio_init(struct s626_struct * s626ptr)
{
	struct s626_subd_dio_private *sdio_priv;
	uint16_t group;
	struct a4l_subdevice *s;

	/*  Prepare to treat writes to wr_cap_sel as capture disables. */
	debi_write(s626ptr, LP_MISC1, MISC1_NOEDCAP);

	/*  For each group of sixteen channels */
	for (group = 0; group < S626_DIO_BANKS; group++) {
		s = a4l_get_subd(s626ptr->dev, group + SUBD_DIO_A);
		sdio_priv = (struct s626_subd_dio_private *) &s->priv;

		/*  Disable all interrupts. */
		debi_write(s626ptr, sdio_priv->wr_int_sel, 0);
		/*  Disable all event  captures. */
		debi_write(s626ptr, sdio_priv->wr_cap_sel, 0xFFFF);
		/*  Init all DIOs to  default edge  polarity */
		debi_write(s626ptr, sdio_priv->wr_edg_sel, 0);
		/*  Program all outputs  to inactive state. */
		debi_write(s626ptr, sdio_priv->wrd_out, 0);
	}
}

static int s626_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	unsigned int *data = (unsigned int *) insn->data;
	struct s626_subd_dio_private *subdpriv;
	int chan = CR_CHAN(insn->chan_desc);
	struct s626_struct * s626ptr = NULL;
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	int group, mask;

	subdpriv = (struct s626_subd_dio_private *) subd->priv;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	group = chan >> 2;
	mask = 0xF << (group << 2);

	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
		subdpriv->state |= 1 << (group + 10);
		subdpriv->io_bits |= mask;
		break;
	case A4L_INSN_CONFIG_DIO_INPUT:
		subdpriv->state &= ~(1 << (group + 10));
		subdpriv->io_bits &= ~mask;
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (subdpriv->io_bits & mask) ? A4L_OUTPUT : A4L_INPUT;
		return 0;
	default:
		return -EINVAL;
	}

	outw(subdpriv->state, ADDR_REG(REG_DIO));

	return 0;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The comedi
 * core can convert between insn_bits and insn_read/write */

static int s626_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	/*
	 * The insn data consists of a mask in data[0] and the new data in
	 * data[1]. The mask defines which bits we are concerning about.
	 * The new data must be anded with the mask.  Each channel
	 * corresponds to a bit.
	 */
	struct s626_subd_dio_private *subdpriv;
	struct s626_struct * s626ptr = NULL;
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;

	uint16_t *data = (uint16_t *) insn->data;
	subdpriv = (struct s626_subd_dio_private *) subd->priv;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (data[0]) {
		subdpriv->state &= ~data[0];
		subdpriv->state |= data[0] & data[1];
		debi_write(s626ptr, subdpriv->wrd_out, subdpriv->state);
	}
	data[1] = debi_read(s626ptr, subdpriv->rdd_in);

	return 0;
}

static int s626_gpct_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	unsigned int *data = (unsigned int *) insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	struct s626_subd_gpct_private *subdpriv;
	struct s626_struct * s626ptr = NULL;
	struct s626_priv *devpriv = NULL;
	struct a4l_device *dev = subd->dev;
	struct list_head *ptr;
	struct enc_private *k;
	uint16_t Setup, valueSrclatch, enab;
	int i;

	subdpriv = (struct s626_subd_gpct_private *) subd->priv;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	a4l_dbg(1, drv_dbg, dev,"s626_gpct_insn_config: Configuring Channel %d\n",
		subdev_channel);

	for (i = 0; i < MAX_GPCT_CONFIG_DATA; i++) {
		subdpriv->config[subdev_channel].data[i] = data[i];
		a4l_dbg(1, drv_dbg, dev, "data[%d]=%x\n", i, data[i]);
	}

	switch (data[0]) {
	case A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER:
		/*  preload upon */
		/*  index. */
		/*  Disable hardware index. */
		/*  Operating mode is Counter. */
		/*  Active high clock. */
		/*  Clock multiplier is 1x. */

		Setup = (LOADSRC_INDX << BF_LOADSRC) |
				 (INDXSRC_SOFT << BF_INDXSRC) |
				 (CLKSRC_COUNTER << BF_CLKSRC) |
				 (CLKPOL_POS << BF_CLKPOL) |
				 (CLKMULT_1X << BF_CLKMULT) |
				 (CLKENAB_INDEX << BF_CLKENAB);

		valueSrclatch = LATCHSRC_AB_READ;
		enab = CLKENAB_ALWAYS;
		k = &(subdpriv->enc_private_data[CR_CHAN(insn->chan_desc)]);

		k->set_mode(s626ptr, k, Setup, TRUE);
		preload(s626ptr, k, data[1]);
		k->pulse_index(s626ptr, k);
		set_latch_source(s626ptr, k, valueSrclatch);
		k->set_enable(s626ptr, k, (uint16_t) (enab != 0));
		break;

	default:
		__a4l_err("s626_gpct_insn_config: unsupported GPCT_insn_config\n");
		return -EINVAL;
		break;
	}

	return 0;

}

static int s626_gpct_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	uint32_t *data = (uint32_t *) insn->data;
	struct s626_subd_gpct_private *subdpriv =
	(struct s626_subd_gpct_private *) subd->priv;
	int i;
	struct enc_private *k =
	&(subdpriv->enc_private_data[CR_CHAN(insn->chan_desc)]);

	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (insn->data_size <= 0) {
		__a4l_err("s626_gpct_rinsn: data size should be > 0\n");
		return -EINVAL;
	}

	for (i = 0; i < insn->data_size / sizeof(uint32_t); i++) {
		data[i] = read_latch(s626ptr, k);
	}

	return 0;
}

static int s626_gpct_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s626_subd_gpct_private *subdpriv = (struct s626_subd_gpct_private *) subd->priv;
	uint32_t *data = (uint32_t *) insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	short value;
	union cmReg cmReg;
	struct s626_priv *devpriv = NULL;
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == subd->dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	a4l_dbg(1, drv_dbg, dev,"s626_gpct_winsn: GPCT_INSN_WRITE on channel %d\n",
		subdev_channel);

	cmReg.value = inw(ADDR_CHAN_REG(REG_C0M, subdev_channel));
	a4l_dbg(1, drv_dbg, dev,"s626_gpct_winsn: Counter Mode Register: %x\n",
		cmReg.value);

	/* Check what Application of Counter this channel is configured for */
	switch (subdpriv->config[subdev_channel].app) {
	case position_measurement:
		a4l_dbg(1, drv_dbg, dev, "s626_gpct_winsn: INSN_WRITE: PM\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H, subdev_channel));
		outw(0xFFFF & (*data), ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case single_pulse_generator:
		a4l_dbg(1, drv_dbg, dev, "s626_gpct_winsn: INSN_WRITE: SPG\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H, subdev_channel));
		outw(0xFFFF & (*data), ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case pulse_train_generation:
		/*
		 * data[0] contains the PULSE_WIDTH
		 * data[1] contains the PULSE_PERIOD
		 * @pre PULSE_PERIOD > PULSE_WIDTH > 0
		 * The above periods must be expressed as a multiple of the
		 * pulse frequency on the selected source
		 */
		a4l_dbg(1, drv_dbg, dev, "s626_gpct_winsn: INSN_WRITE: PTG\n");
		if ((data[1] > data[0]) && (data[0] > 0)) {
			(subdpriv->config[subdev_channel]).data[0] = data[0];
			(subdpriv->config[subdev_channel]).data[1] = data[1];
		}
		else {
			__a4l_err("s626_gpct_winsn: INSN_WRITE: PTG: "
				  "Problem with Pulse params -> %du %du\n",
				  data[0], data[1]);
			return -EINVAL;
		}

		value = (short) ((*data >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));
		value = (short) (*data & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;
	default:
		__a4l_err(
			 "s626_gpct_winsn: INSN_WRITE: "
			 "Functionality %d not implemented yet\n",
			 subdpriv->config[subdev_channel].app);
		return -EINVAL;
	}

	return 0;
}

/* --- Channels descriptor --- */

static struct a4l_channels_desc s626_chan_desc_ai = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_AI_CHANS,
	.chans ={ { A4L_CHAN_AREF_GROUND, S626_AI_BITS },},
};

static struct a4l_channels_desc s626_chan_desc_ao = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_AO_CHANS,
	.chans = { { A4L_CHAN_AREF_GROUND, S626_AO_BITS },},
};

static struct a4l_channels_desc s626_chan_desc_dioA = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_DIO_CHANS,
	.chans = { { A4L_CHAN_AREF_GROUND, S626_DIO_BITS },},
};

static struct a4l_channels_desc s626_chan_desc_dioB = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_DIO_CHANS,
	.chans = { { A4L_CHAN_AREF_GROUND, S626_DIO_BITS },},
};

static struct a4l_channels_desc s626_chan_desc_dioC = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_DIO_CHANS,
	.chans = { { A4L_CHAN_AREF_GROUND, S626_DIO_BITS },},
};

static struct a4l_channels_desc s626_chan_desc_gpct = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S626_GPCT_CHANS,
	.chans = {{ A4L_CHAN_AREF_GROUND, S626_GPCT_BITS },},
};

/* --- Subdevice initialization functions --- */

/* Analog input subdevice */
static void setup_subd_ai(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AI | A4L_SUBD_MASK_READ;
	subd->chan_desc = &s626_chan_desc_ai;
	subd->rng_desc = &a4l_range_bipolar10;

	subd->insn_config = s626_ai_insn_config;
	subd->insn_read = s626_ai_insn_read;
	subd->do_cmd = s626_ai_cmd;
	subd->do_cmdtest = s626_ai_cmdtest;
	subd->cancel = s626_ai_cancel;
}

/* Analog output subdevice */
static void setup_subd_ao(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AO | A4L_SUBD_MASK_WRITE;
	subd->chan_desc = &s626_chan_desc_ao;
	subd->rng_desc = &a4l_range_bipolar10;

	subd->insn_write = s626_ao_winsn;
	subd->insn_read = s626_ao_rinsn;

	memset(&(subd->priv), 0, sizeof(struct s626_subd_ao_private));
}

/* Digital i/o subdevice */
static void setup_subd_dioA(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &s626_chan_desc_dioA;
	subd->rng_desc = &range_digital;
	subd->insn_bits = s626_dio_insn_bits;
	subd->insn_config = s626_dio_insn_config;
	memcpy(&(subd->priv), (const void *) (&s626_subd_dio_private_A),
	       sizeof(struct s626_subd_dio_private));
}
static void setup_subd_dioB(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &s626_chan_desc_dioB;
	subd->rng_desc = &range_digital;
	subd->insn_bits = s626_dio_insn_bits;
	subd->insn_config = s626_dio_insn_config;
	memcpy(&(subd->priv), (const void *) (&s626_subd_dio_private_B),
	       sizeof(struct s626_subd_dio_private));
}
static void setup_subd_dioC(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &s626_chan_desc_dioC;
	subd->rng_desc = &range_digital;
	subd->insn_bits = s626_dio_insn_bits;
	subd->insn_config = s626_dio_insn_config;
	memcpy(&(subd->priv), (const void *) (&s626_subd_dio_private_C),
	       sizeof(struct s626_subd_dio_private));
}

/* General purpose counter/timer (gpct) */
static void setup_subd_gpct(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_COUNTER;
	subd->chan_desc = &s626_chan_desc_gpct;
	subd->insn_read = s626_gpct_rinsn;
	subd->insn_config = s626_gpct_insn_config;
	subd->insn_write = s626_gpct_winsn;

	//only the first field of the private structure for this subdevice
	memcpy(&(subd->priv), (const void *) (&enc_private_data_init[0]),
	       sizeof(struct enc_private) * S626_GPCT_CHANS);
}

static struct setup_subd setup_subds[6] = {
	[0] = { .setup_func = setup_subd_ai,
		.sizeof_priv = sizeof(struct s626_subd_ai_private), },
	[1] = {
		.setup_func = setup_subd_ao,
		.sizeof_priv = sizeof(struct s626_subd_ao_private), },
	[2] ={
		.setup_func = setup_subd_dioA,
		.sizeof_priv = sizeof(struct s626_subd_dio_private), },
	[3] = {
		.setup_func = setup_subd_dioB,
		.sizeof_priv = sizeof(struct s626_subd_dio_private), },
	[4] = {
		.setup_func = setup_subd_dioC,
		.sizeof_priv = sizeof(struct s626_subd_dio_private), },
	[5] = {
		.setup_func = setup_subd_gpct,
		.sizeof_priv = sizeof(struct s626_subd_gpct_private), },
};

static void counters_init(struct s626_struct * s626ptr)
{
	struct s626_subd_gpct_private *sgpct_priv;
	struct enc_private *k;
	struct a4l_subdevice *t;
	int chan;

	/*  preload upon
	 *  Disable hardware index.
	 *  Operating mode is counter.
	 *  Active high clock.
	 *  Count direction is up.
	 *  Clock multiplier is 1x.
	 *  Enabled by index
	 */

	uint16_t Setup = (LOADSRC_INDX << BF_LOADSRC) |
			 (INDXSRC_SOFT << BF_INDXSRC) |
			 (CLKSRC_COUNTER << BF_CLKSRC) |
			 (CLKPOL_POS << BF_CLKPOL) |
			 (CNTDIR_UP << BF_CLKPOL) |
			 (CLKMULT_1X << BF_CLKMULT) |
			 (CLKENAB_INDEX << BF_CLKENAB);

	/* Disable all counter interrupts and clear any captured counter events. */
	for (chan = 0; chan < S626_ENCODER_CHANNELS; chan++) {

		t = a4l_get_subd(s626ptr->dev, SUBD_ENC);
		sgpct_priv = (struct s626_subd_gpct_private *)t->priv;

		k = &sgpct_priv->enc_private_data[chan];

		k->set_mode(s626ptr, k, Setup, TRUE);
		k->set_int_src(s626ptr, k, 0);
		k->reset_cap_flags(s626ptr, k);
		k->set_enable(s626ptr, k, CLKENAB_ALWAYS);
	}
}

static void s626_initialize(struct s626_struct * s626ptr)
{
	struct s626_priv *devpriv = s626ptr->private;
	dma_addr_t pPhysBuf;
	uint16_t chan;
	int i;

	/* Enable DEBI and audio pins, enable I2C interface */
	MC_ENABLE(P_MC1, MC1_DEBI | MC1_AUDIO | MC1_I2C);

	/*
	 *  Configure DEBI operating mode
	 *
	 *   Local bus is 16 bits wide
	 *   Declare DEBI transfer timeout interval
	 *   Set up byte lane steering
	 *   Intel-compatible local bus (DEBI never times out)
	 */
	WR7146(P_DEBICFG, DEBI_CFG_SLAVE16 | (DEBI_TOUT << DEBI_CFG_TOUT_BIT)
	       | DEBI_SWAP | DEBI_CFG_INTEL);

	/* Disable MMU paging */
	WR7146(P_DEBIPAGE, DEBI_PAGE_DISABLE);

	/* Init GPIO so that ADC Start* is negated */
	WR7146(P_GPIO, GPIO_BASE | GPIO1_HI);

	/* I2C device address for onboard eeprom (revb) */
	devpriv->I2CAdrs = 0xA0;

	/*
	 * Issue an I2C ABORT command to halt any I2C
	 * operation in progress and reset BUSY flag.
	 */
	WR7146(P_I2CSTAT, I2C_CLKSEL | I2C_ABORT);
	MC_ENABLE(P_MC2, MC2_UPLD_IIC);
	while ((RR7146(P_MC2) & MC2_UPLD_IIC) == 0)
		rtdm_task_sleep(2000);

	/*
	 * Per SAA7146 data sheet, write to STATUS
	 * reg twice to reset all  I2C error flags.
	 */
	for (i = 0; i < 2; i++) {
		WR7146(P_I2CSTAT, I2C_CLKSEL);
		MC_ENABLE(P_MC2, MC2_UPLD_IIC);
		while (!MC_TEST(P_MC2, MC2_UPLD_IIC))
			rtdm_task_sleep(2000);
	}

	/*
	 * Init audio interface functional attributes: set DAC/ADC
	 * serial clock rates, invert DAC serial clock so that
	 * DAC data setup times are satisfied, enable DAC serial
	 * clock out.
	 */
	WR7146(P_ACON2, ACON2_INIT);

	/*
	 * Set up TSL1 slot list, which is used to control the
	 * accumulation of ADC data: RSD1 = shift data in on SD1.
	 * SIB_A1  = store data uint8_t at next available location
	 * in FB BUFFER1 register.
	 */
	WR7146(P_TSL1, RSD1 | SIB_A1);
	WR7146(P_TSL1 + 4, RSD1 | SIB_A1 | EOS);

	/* Enable TSL1 slot list so that it executes all the time */
	WR7146(P_ACON1, ACON1_ADCSTART);

	/*
	 * Initialize RPS registers used for ADC
	 */

	/* Physical start of RPS program */
	WR7146(P_RPSADDR1, (uint32_t)devpriv->RPSBuf.PhysicalBase);
	/* RPS program performs no explicit mem writes */
	WR7146(P_RPSPAGE1, 0);
	/* Disable RPS timeouts */
	WR7146(P_RPS1_TOUT, 0);

	/*
	 * Initialize the DAC interface
	 */

	/*
	 * Init Audio2's output DMAC attributes:
	 *   burst length = 1 DWORD
	 *   threshold = 1 DWORD.
	 */
	WR7146(P_PCI_BT_A, 0);

	/*
	 * Init Audio2's output DMA physical addresses.  The protection
	 * address is set to 1 DWORD past the base address so that a
	 * single DWORD will be transferred each time a DMA transfer is
	 * enabled.
	 */
	pPhysBuf = devpriv->ANABuf.PhysicalBase + (DAC_WDMABUF_OS * sizeof(uint32_t));
	WR7146(P_BASEA2_OUT, (uint32_t) pPhysBuf);
	WR7146(P_PROTA2_OUT, (uint32_t) (pPhysBuf + sizeof(uint32_t)));

	/*
	 * Cache Audio2's output DMA buffer logical address.  This is
	 * where DAC data is buffered for A2 output DMA transfers.
	 */
	devpriv->pDacWBuf = (uint32_t *) devpriv->ANABuf.LogicalBase + DAC_WDMABUF_OS;

	/*
	 * Audio2's output channels does not use paging.  The
	 * protection violation handling bit is set so that the
	 * DMAC will automatically halt and its PCI address pointer
	 * will be reset when the protection address is reached.
	 */
	WR7146(P_PAGEA2_OUT, 8);

	/*
	 * Initialize time slot list 2 (TSL2), which is used to control
	 * the clock generation for and serialization of data to be sent
	 * to the DAC devices.  Slot 0 is a NOP that is used to trap TSL
	 * execution; this permits other slots to be safely modified
	 * without first turning off the TSL sequencer (which is
	 * apparently impossible to do).  Also, SD3 (which is driven by a
	 * pull-up resistor) is shifted in and stored to the MSB of
	 * FB_BUFFER2 to be used as evidence that the slot sequence has
	 * not yet finished executing.
	 */

	/* Slot 0: Trap TSL execution, shift 0xFF into FB_BUFFER2 */
	SETVECT(0, XSD2 | RSD3 | SIB_A2 | EOS);

	/*
	 * Initialize slot 1, which is constant.  Slot 1 causes a
	 * DWORD to be transferred from audio channel 2's output FIFO
	 * to the FIFO's output buffer so that it can be serialized
	 * and sent to the DAC during subsequent slots.  All remaining
	 * slots are dynamically populated as required by the target
	 * DAC device.
	 */

	/* Slot 1: Fetch DWORD from Audio2's output FIFO */
	SETVECT(1, LF_A2);

	/* Start DAC's audio interface (TSL2) running */
	WR7146(P_ACON1, ACON1_DACSTART);

	/*
	 * Init Trim DACs to calibrated values.  Do it twice because the
	 * SAA7146 audio channel does not always reset properly and
	 * sometimes causes the first few TrimDAC writes to malfunction.
	 */
	load_trim_dacs(s626ptr);
	load_trim_dacs(s626ptr);

	/*
	 * Manually init all gate array hardware in case this is a soft
	 * reset (we have no way of determining whether this is a warm
	 * or cold start).  This is necessary because the gate array will
	 * reset only in response to a PCI hard reset; there is no soft
	 * reset function.
	 */

	/*
	 * Init all DAC outputs to 0V and init all DAC setpoint and
	 * polarity images.
	 */
	for (chan = 0; chan < S626_DAC_CHANNELS; chan++)
		set_adc(s626ptr, chan, 0);

	/* Init counters */
	counters_init(s626ptr);

	/*
	 * Without modifying the state of the Battery Backup enab, disable
	 * the watchdog timer, set DIO channels 0-5 to operate in the
	 * standard DIO (vs. counter overflow) mode, disable the battery
	 * charger, and reset the watchdog interval selector to zero.
	 */
	write_misc2(s626ptr, debi_read(s626ptr, LP_RDMISC2) & MISC2_BATT_ENABLE);

	/* Initialize the digital I/O subsystem */
	s626_dio_init(s626ptr);

}

static int dev_s626_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{

	int i, err = 0;
	struct s626_struct * s626ptr = NULL;

	a4l_info(dev, "Attaching s626\n");

	s626ptr = kmalloc(sizeof(struct s626_struct), GFP_KERNEL);
	if (s626ptr == NULL) {
		__a4l_err("No memory for s626 structure\n");
		return -ENOMEM;
	}

	memset(s626ptr, 0x00, sizeof(struct s626_struct));
	s626ptr->dev = dev;

	rtdm_lock_init(&s626ptr->lock);

	s626ptr->private = kmalloc(sizeof(struct s626_priv), GFP_KERNEL);
	if (s626ptr->private == NULL) {
		__a4l_err("No memory for s626 private data\n");
		return -ENOMEM;
	}

	list_add(&(s626ptr->list), &s626_list);

	err = pci_register_driver(&drv_pci_s626);
	if (err < 0) {
		__a4l_err("pci_register_driver = %d\n", err);
		return -ENOMEM;
	}

	pci_set_master(s626ptr->pcidev);
	err = pci_request_regions(s626ptr->pcidev, dev->driver->board_name);
	if (err < 0) {
		__a4l_err( "Can't request regions\n");
		pci_disable_device(s626ptr->pcidev);
		return -ENOMEM;
	}

	s626ptr->private->base_addr = ioremap(pci_resource_start( s626ptr->pcidev, 0),
					      pci_resource_len( s626ptr->pcidev, 0));

	if (s626ptr->private->base_addr == NULL)
		return -ENOMEM;

	rtdm_event_init(&s626ptr->private->synch, 0);

	/* disable master interrupt */
	writel(0, s626ptr->private->base_addr + P_IER);

	/* soft reset */
	writel(MC1_SOFT_RESET, s626ptr->private->base_addr + P_MC1);

	/* DMA FIXME DMA */
	err = s626_allocate_dma_buffers(s626ptr);
	if (err)
		return err;

	/* create irq service as a spearate rt task */
	err = rtdm_task_init(&s626ptr->irq_service_task, "s626_irq_service",
			     s626_irq_service, s626ptr, 99, 0);
	if (err)
		return err;

	if (s626ptr->pcidev->irq) {
		err = a4l_request_irq(dev, s626ptr->pcidev->irq,
				      s626_irq_handler,
				      RTDM_IRQTYPE_SHARED, dev);
		if (err < 0)
			return err;
	}

	/* Allocate the subdevice structures. */
	for (i = 0; i < 6; ++i) {
		struct a4l_subdevice *subd = a4l_alloc_subd(setup_subds[i].sizeof_priv,
						  setup_subds[i].setup_func);
		if (subd == NULL)
			return -ENOMEM;

		err = a4l_add_subd(dev, subd);
		if (err != i)
			return err;
	}

	s626_initialize(s626ptr);

	return 0;
}

static int dev_s626_detach(struct a4l_device *dev)
{
	struct list_head *ptr;
	struct s626_struct * s626ptr = NULL;
	struct s626_priv *devpriv = NULL;

	a4l_info(dev, "Detaching s626\n");

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->dev == dev) {
			devpriv = s626ptr->private;
			break;
		}
	}

	if (devpriv == NULL) {
		__a4l_err( "Inside %s, error, no device\n", __PRETTY_FUNCTION__);
		return -ENODEV;
	}

	if (s626ptr->private) {
		/* stop ai_command */
		s626ptr->private->ai_cmd_running = 0;

		if (s626ptr->private->base_addr) {
			/* interrupt mask */
			WR7146(P_IER, 0);
			/*  Disable master interrupt. */
			WR7146(P_ISR, IRQ_GPIO3 | IRQ_RPS1);
			/*  Clear board's IRQ status flag. */

			/*  Disable the watchdog timer and battery charger. */
			write_misc2(s626ptr, 0);

			/*  Close all interfaces on 7146 device. */
			WR7146(P_MC1, MC1_SHUTDOWN);
			WR7146(P_ACON1, ACON1_BASE);

			close_dmab(s626ptr, &s626ptr->private->RPSBuf, DMABUF_SIZE);
			close_dmab(s626ptr, &s626ptr->private->ANABuf, DMABUF_SIZE);
		}

		if (a4l_get_irq(dev) != A4L_IRQ_UNUSED)
			a4l_free_irq(dev, a4l_get_irq(dev));

		if (s626ptr->private->base_addr)
			iounmap(s626ptr->private->base_addr);
	}

	s626ptr->terminate_task = 1;
	rtdm_task_join_nrt(&s626ptr->irq_service_task, 200);

	pci_release_regions(s626ptr->pcidev);
	pci_disable_device(s626ptr->pcidev);

	pci_unregister_driver(&drv_pci_s626);

	if (s626ptr->private)
		kfree(s626ptr->private);

	list_del(&(s626ptr->list));

	kfree(s626ptr);

	return 0;
}

static int s626_pci_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct s626_struct * s626ptr = NULL;
	struct list_head *ptr;
	int err = 0;

	if (pci_enable_device(dev) < 0) {
		__a4l_err("error enabling s626\n");
		err = -EIO;
	}

	list_for_each( ptr, &s626_list) {
		s626ptr = list_entry(ptr, struct s626_struct, list);
		if (s626ptr->pcidev == NULL)
			s626ptr->pcidev = dev;
	}

	return err;
}

static void s626_pci_remove(struct pci_dev *dev)
{

}

static struct a4l_driver drv_s626 = {
	.owner = THIS_MODULE,
	.board_name = "S626",
	.attach = dev_s626_attach,
	.detach = dev_s626_detach,
	.privdata_size = sizeof(struct s626_priv),
};

static int __init drv_s626_init(void)
{
	return a4l_register_drv(&drv_s626);
}

static void __exit drv_s626_cleanup(void)
{
	a4l_unregister_drv(&drv_s626);
}

MODULE_DESCRIPTION("Analogy driver for Sensoray Model 626 board.");
MODULE_LICENSE("GPL");

module_init(drv_s626_init);
module_exit(drv_s626_cleanup);


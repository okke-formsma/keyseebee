#![no_main]
#![no_std]

// set the panic handler
use panic_halt as _;

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use generic_array::typenum::{U4, U6};
use hal::gpio::{gpioa, gpiob, Floating, Input, Output, PullUp, PushPull};
use hal::prelude::*;
use hal::serial;
use hal::usb;
use hal::{stm32, timers};
use keyberon::action::{k, l, m, Action, Action::*, HoldTapConfig};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::KbHidReport;
use keyberon::key_code::KeyCode::*;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::{Matrix, PressedKeys};
use nb::block;
use rtic::app;
use stm32f0xx_hal as hal;
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::device::UsbDeviceState;

type UsbClass = keyberon::Class<'static, usb::UsbBusType, ()>;
type UsbDevice = usb_device::device::UsbDevice<'static, usb::UsbBusType>;

trait ResultExt<T> {
    fn get(self) -> T;
}
impl<T> ResultExt<T> for Result<T, Infallible> {
    fn get(self) -> T {
        match self {
            Ok(v) => v,
            Err(e) => match e {},
        }
    }
}

pub struct Cols(
    gpioa::PA15<Input<PullUp>>,
    gpiob::PB3<Input<PullUp>>,
    gpiob::PB4<Input<PullUp>>,
    gpiob::PB5<Input<PullUp>>,
    gpiob::PB8<Input<PullUp>>,
    gpiob::PB9<Input<PullUp>>,
);
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = Infallible>,
    U6,
    [0, 1, 2, 3, 4, 5]
}

pub struct Rows(
    gpiob::PB0<Output<PushPull>>,
    gpiob::PB1<Output<PushPull>>,
    gpiob::PB2<Output<PushPull>>,
    gpiob::PB10<Output<PushPull>>,
);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = Infallible>,
    U4,
    [0, 1, 2, 3]
}

const L1_ENTER: Action = HoldTap {
    timeout: 200,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: &l(1),
    tap: &k(Enter),
};

macro_rules! s {
    ($k:ident) => {
        m(&[LShift, $k])
    };
}

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    &[
        &[Trans,Trans,Trans,Trans,Trans,Trans,k(Tab),k(Kb7),k(Kb8),k(Kb9),k(Up),s!(Equal) ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,s!(Kb8),k(Kb4),k(Kb5),k(Kb6),k(Down),k(Minus)   ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,k(Slash),k(Kb1),k(Kb2),k(Kb3),k(Left),k(Right)  ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,L1_ENTER,k(Dot),k(Kb0),k(BSpace),Trans,   Trans      ],
    ],
    &[
        &[Trans,Trans,Trans,Trans,Trans,Trans,k(Tab),k(Home),k(Up),k(PgUp),k(Up),s!(Equal) ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,s!(Kb8),k(Left),k(Down),k(Right),k(Down),k(Minus)   ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,k(Slash),k(End),k(Down),k(PgDown),k(Left),k(Right)  ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,k(Dot),k(Kb0),k(Delete),Trans,   Trans      ],
    ]
];

#[app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice,
        usb_class: UsbClass,
        matrix: Matrix<Cols, Rows>,
        debouncer: Debouncer<PressedKeys<U4, U6>>,
        layout: Layout,
        timer: timers::Timer<stm32::TIM3>,
        transform: fn(Event) -> Event,
        tx: serial::Tx<hal::pac::USART1>,
        rx: serial::Rx<hal::pac::USART1>,
    }

    #[init]
    fn init(mut c: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<usb::UsbBusType>> = None;

        let mut rcc = c
            .device
            .RCC
            .configure()
            .hsi48()
            .enable_crs(c.device.CRS)
            .sysclk(48.mhz())
            .pclk(24.mhz())
            .freeze(&mut c.device.FLASH);

        let gpioa = c.device.GPIOA.split(&mut rcc);
        let gpiob = c.device.GPIOB.split(&mut rcc);

        let usb = usb::Peripheral {
            usb: c.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };
        *USB_BUS = Some(usb::UsbBusType::new(usb));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_class = keyberon::new_class(usb_bus, ());
        let usb_dev = keyberon::new_device(usb_bus);

        let mut timer = timers::Timer::tim3(c.device.TIM3, 1.khz(), &mut rcc);
        timer.listen(timers::Event::TimeOut);

        let pb12: &gpiob::PB12<Input<Floating>> = &gpiob.pb12;
        let is_left = pb12.is_low().get();
        let transform: fn(Event) -> Event = if is_left {
            |e| e
        } else {
            |e| e.transform(|i, j| (i, 11 - j))
        };

        let (pa9, pa10) = (gpioa.pa9, gpioa.pa10);
        let pins = cortex_m::interrupt::free(move |cs| {
            (pa9.into_alternate_af1(cs), pa10.into_alternate_af1(cs))
        });
        let mut serial = serial::Serial::usart1(c.device.USART1, pins, 38_400.bps(), &mut rcc);
        serial.listen(serial::Event::Rxne);
        let (tx, rx) = serial.split();

        let pa15 = gpioa.pa15;
        let matrix = cortex_m::interrupt::free(move |cs| {
            Matrix::new(
                Cols(
                    pa15.into_pull_up_input(cs),
                    gpiob.pb3.into_pull_up_input(cs),
                    gpiob.pb4.into_pull_up_input(cs),
                    gpiob.pb5.into_pull_up_input(cs),
                    gpiob.pb8.into_pull_up_input(cs),
                    gpiob.pb9.into_pull_up_input(cs),
                ),
                Rows(
                    gpiob.pb0.into_push_pull_output(cs),
                    gpiob.pb1.into_push_pull_output(cs),
                    gpiob.pb2.into_push_pull_output(cs),
                    gpiob.pb10.into_push_pull_output(cs),
                ),
            )
        });

        init::LateResources {
            usb_dev,
            usb_class,
            timer,
            debouncer: Debouncer::new(PressedKeys::default(), PressedKeys::default(), 5),
            matrix: matrix.get(),
            layout: Layout::new(LAYERS),
            transform,
            tx,
            rx,
        }
    }

    #[task(binds = USART1, priority = 5, spawn = [handle_event], resources = [rx])]
    fn rx(c: rx::Context) {
        static mut BUF: [u8; 4] = [0; 4];

        if let Ok(b) = c.resources.rx.read() {
            BUF.rotate_left(1);
            BUF[3] = b;

            if BUF[3] == b'\n' {
                if let Ok(event) = de(&BUF[..]) {
                    c.spawn.handle_event(Some(event)).unwrap();
                }
            }
        }
    }

    #[task(binds = USB, priority = 4, resources = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        if c.resources.usb_dev.poll(&mut [c.resources.usb_class]) {
            c.resources.usb_class.poll();
        }
    }

    #[task(priority = 3, capacity = 8, resources = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<Event>) {
        let report: KbHidReport = match event {
            None => c.resources.layout.tick().collect(),
            Some(e) => {
                c.resources.layout.event(e);
                return;
            }
        };
        if !c
            .resources
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.resources.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.resources.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(
        binds = TIM3,
        priority = 2,
        spawn = [handle_event],
        resources = [matrix, debouncer, timer, &transform, tx],
    )]
    fn tick(c: tick::Context) {
        c.resources.timer.wait().ok();

        for event in c
            .resources
            .debouncer
            .events(c.resources.matrix.get().get())
            .map(c.resources.transform)
        {
            for &b in &ser(event) {
                block!(c.resources.tx.write(b)).get();
            }
            c.spawn.handle_event(Some(event)).unwrap();
        }
        c.spawn.handle_event(None).unwrap();
    }

    extern "C" {
        fn CEC_CAN();
    }
};

fn de(bytes: &[u8]) -> Result<Event, ()> {
    match *bytes {
        [b'P', i, j, b'\n'] => Ok(Event::Press(i, j)),
        [b'R', i, j, b'\n'] => Ok(Event::Release(i, j)),
        _ => Err(()),
    }
}
fn ser(e: Event) -> [u8; 4] {
    match e {
        Event::Press(i, j) => [b'P', i, j, b'\n'],
        Event::Release(i, j) => [b'R', i, j, b'\n'],
    }
}

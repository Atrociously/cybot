use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use quote::quote;
use syn::ItemFn;


#[proc_macro_attribute]
pub fn entry(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let item = syn::parse_macro_input!(item as ItemFn);

    let fn_ident = &item.sig.ident;
    let wrap_ident = Ident::new(&format!("_cybot_run_{}", item.sig.ident), Span::call_site());

    quote! {
        #item

        #[allow(non_snake_case)]
        #[cybot::cortex_m_rt::entry]
        fn #wrap_ident() -> ! {
            ::cybot::run(#fn_ident)
        }
    }.into()
}

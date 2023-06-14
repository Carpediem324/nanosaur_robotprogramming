"============================================
" VIMRC
" made by SungTae Moon
"============================================

set ic
set nu
set softtabstop=4                              " TAB키를 눌렀을때 몇 칸을 이동
set tabstop=4                                  " 하나의 TAB을 몇 칸으로 인식
set shiftwidth=4                               " <<, >>을 눌렀을 때 몇 칸을 이동
set expandtab                                  " TAB을 space로 인식
set formatoptions=croql
set ruler
set showmode
set smartindent
set hlsearch



"set tags=[tags 위치 디렉토리]
"set tags=/home/stmoon/tags

syntax on

map <F1> K<cr>
map <F2> :w<cr>
map <F3> :make<cr>
map <F4> :q <cr>
map <F5> :exec '!python3' shellescape(@%, 1)<cr>
map <F9> :exec '!python3' shellescape(@%, 1)<cr>
"map :cn
"map :bn
"map :bp

"=====================================
" VIM Color
"=====================================
hi Comment ctermfg=DarkCyan
hi Statement ctermfg=DarkGreen
hi Label ctermfg=DarkGreen
hi Conditional ctermfg=DarkGreen
hi String ctermfg=DarkYellow
hi Repeat ctermfg=DarkYellow
hi Format ctermfg=DarkYellow
hi Character ctermfg=DarkYellow
hi Special ctermfg=DarkYellow
hi Constant ctermfg=DarkRed


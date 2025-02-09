"#####表示設定#####

set iminsert=0  "デフォルトで挿入モード0でオフ
set imsearch=-1
set number "行番号を表示する
set title "編集中のファイル名を表示
set showmatch "括弧入力時の対応する括弧を表示
syntax on "コードの色分け
set tabstop=2
set autoindent
set expandtab
"retab=2
set shiftwidth=2

"#####検索設定#####
"set ignorecase "大文字/小文字の区別なく検索する
"set smartcase "検索文字列に大文字が含まれている場合は区別して検索する
set wrapscan "検索時に最後まで行ったら最初に戻る
set hlsearch "hilight
"---- vi追記 ----
set encoding=utf-8
set fileencodings=utf-8
set fileformats=unix,dos,mac
set nowrap              "折り返し
set pumheight=3
"色設定
set background=dark
hi Comment ctermfg=2

"colorscheme elflord

set tags=./tags;,./TAGS,tags,TAGS

highlight StatusLine guifg=black guibg=white gui=none ctermfg=white ctermbg=darkgray cterm=none

""""""""""""""""""""""""""""""
"挿入モード時、ステータスラインの色を変更
""""""""""""""""""""""""""""""
let g:hi_insert = 'highlight StatusLine guifg=darkblue guibg=darkyellow gui=none ctermfg=darkred ctermbg=darkgray cterm=none'

	if has('syntax')
		augroup InsertHook
			autocmd!
			autocmd InsertEnter * call s:StatusLine('Enter')
			autocmd InsertLeave * call s:StatusLine('Leave')
			augroup END
			endif

			let s:slhlcmd = ''
			function! s:StatusLine(mode)
			if a:mode == 'Enter'
				silent! let s:slhlcmd = 'highlight ' . s:GetHighlight('StatusLine')
					silent exec g:hi_insert
			else
				highlight clear StatusLine
					silent exec s:slhlcmd
					endif
					endfunction

					function! s:GetHighlight(hi)
					redir => hl
					exec 'highlight '.a:hi
					redir END
					let hl = substitute(hl, '[\r\n]', '', 'g')
					let hl = substitute(hl, 'xxx', '', '')
					return hl
					endfunction


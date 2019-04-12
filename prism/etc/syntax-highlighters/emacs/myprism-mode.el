;;; myprism-mode.el --- sample major mode for editing .prism files. -*- coding: utf-8; lexical-binding: t; -*-

;; Copyright © 2019

;; Author: Fatma Faruq
;; Adapted from: http://ergoemacs.org/emacs/elisp_syntax_coloring.html

;; This file is not part of GNU Emacs.

;;; License:

;; You can redistribute this program and/or modify it under the terms of the GNU General Public License version 2.

;;; Commentary:

;; short description here

;; full doc on how to use here

;;; Code:

;; create the list for font-lock.
;; each category of keyword is given a particular face
(setq myprism-font-lock-keywords
      (let* (
            ;; define several category of keywords
            (x-keywords '("module" "endmodule" "init" "A" "clock" "const" "C" "E" "endinit" "endinvariant" "endrewards" "endsystem" "formula" "filter" "func" "F" "global" "G" "invariant" "I" "max" "min" "X" "rewards" "system" "U" "W"))
            (x-types '("dtmc" "probabilistic" "mdp" "nondeterministic" "ctmc" "stochastic" "pta" "bool" "double" "int" "label"))
            (x-constants '("false" "true" ))
            ;;(x-events '(""))
            (x-functions '("Pmax" "Pmin" "P" "prob" "Rmin" "Rmax" "prob" "rate"))

            ;; generate regex string for each category of keywords
            (x-keywords-regexp (regexp-opt x-keywords 'words))
            (x-types-regexp (regexp-opt x-types 'words))
            (x-constants-regexp (regexp-opt x-constants 'words))
            ;;(x-events-regexp (regexp-opt x-events 'words))
            (x-functions-regexp (regexp-opt x-functions 'words)))

        `(
          (,x-types-regexp . font-lock-type-face)
          (,x-constants-regexp . font-lock-constant-face)
          ;;(,x-events-regexp . font-lock-builtin-face)
          (,x-functions-regexp . font-lock-function-name-face)
          (,x-keywords-regexp . font-lock-keyword-face)
          ;; note: order above matters, because once colored, that part won't change.
          ;; in general, put longer words first
          )))

;;;###autoload
(define-derived-mode myprism-mode c-mode "Prism mode"
  "Major mode for editing PRISM Language…"

  ;; code for syntax highlighting
  (setq font-lock-defaults '((myprism-font-lock-keywords))))



;; add the mode to the `features' list
(provide 'myprism-mode)

;;; myprism-mode.el ends here

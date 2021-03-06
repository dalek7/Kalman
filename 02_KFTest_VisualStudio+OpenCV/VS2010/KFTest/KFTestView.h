
// KFTestView.h : interface of the CKFTestView class
//

#pragma once

#include "resource.h"

#include "CVWnd.h"
#include "afxwin.h"
#include "afxcmn.h"


class CKFTestView : public CFormView
{
protected: // create from serialization only
	CKFTestView();
	DECLARE_DYNCREATE(CKFTestView)

public:
	enum{ IDD = IDD_KFTEST_FORM };

// Attributes
public:
	CKFTestDoc* GetDocument() const;

// Operations
public:
	CVWnd	m_wnd1;
	CVWnd*	pWnd1;
// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnInitialUpdate(); // called first time after construct

// Implementation
public:
	virtual ~CKFTestView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	CButton m_chk_run;
	CButton m_chk_model_cv;
	afx_msg void OnBnClickedButton1();
	CSliderCtrl m_sld_vel;
	CStatic m_info1;
	CStatic m_info2;
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	
	CButton m_btn_clear;
};

#ifndef _DEBUG  // debug version in KFTestView.cpp
inline CKFTestDoc* CKFTestView::GetDocument() const
   { return reinterpret_cast<CKFTestDoc*>(m_pDocument); }
#endif


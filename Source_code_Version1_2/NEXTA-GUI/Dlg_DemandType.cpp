// Dlg_DemandType.cpp : implementation file
//

#include "stdafx.h"
#include "TLite.h"
#include "Dlg_DemandType.h"


// CDlg_DemandType dialog

IMPLEMENT_DYNCREATE(CDlg_DemandType, CDHtmlDialog)

CDlg_DemandType::CDlg_DemandType(CWnd* pParent /*=NULL*/)
	: CDHtmlDialog(CDlg_DemandType::IDD, CDlg_DemandType::IDH, pParent)
{

}

CDlg_DemandType::~CDlg_DemandType()
{
}

void CDlg_DemandType::DoDataExchange(CDataExchange* pDX)
{
	CDHtmlDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_DemandTypeLIST, m_DemandTypeGrid);
}

BOOL CDlg_DemandType::OnInitDialog()
{
	CDHtmlDialog::OnInitDialog();

		std::vector<std::string> m_Column_names;

	m_Column_names.push_back ("No.");
	m_Column_names.push_back ("Demand Type");
	m_Column_names.push_back ("Avg VOT");
	m_Column_names.push_back ("Pricing Type");
	m_Column_names.push_back ("% of Pretrip Info");
	m_Column_names.push_back ("% of Enroutetrip Info");


	for(int vt = 0; vt < m_pDoc->m_VehicleTypeVector.size(); vt++)
	{
		DTAVehicleType element = m_pDoc->m_VehicleTypeVector[vt];
		char str_vehicle_type[100];

		sprintf_s(str_vehicle_type,"%% of %s", element.vehicle_type_name );
		m_Column_names.push_back(str_vehicle_type);

	}

	//Add Columns and set headers
	for (size_t i=0;i<m_Column_names.size();i++)
	{

		CGridColumnTrait* pTrait = NULL;
//		pTrait = new CGridColumnTraitEdit();
		m_DemandTypeGrid.InsertColumnTrait((int)i,m_Column_names.at(i).c_str(),LVCFMT_LEFT,-1,-1, pTrait);
		m_DemandTypeGrid.SetColumnWidth((int)i,LVSCW_AUTOSIZE_USEHEADER);
	}
	m_DemandTypeGrid.SetColumnWidth(0, 80);
      

	int FirstDemandTypeIndex= -1;
	int i = 0;
	for(std::vector<DTADemandType>::iterator itr = m_pDoc->m_DemandTypeVector.begin(); itr != m_pDoc->m_DemandTypeVector.end(); itr++, i++)
		{
			// can be also enhanced to edit the real time information percentage
		char text[100];
		sprintf_s(text, "%d",(*itr).demand_type );
		int Index = m_DemandTypeGrid.InsertItem(LVIF_TEXT,i,text , 0, 0, 0, NULL);

		if(FirstDemandTypeIndex<0)
			FirstDemandTypeIndex = Index;  // initiailize the first demand


		sprintf_s(text, "%s",(*itr).demand_type_name);
		m_DemandTypeGrid.SetItemText(Index,1,text);

		sprintf_s(text, "%5.2f",(*itr).average_VOT);
		m_DemandTypeGrid.SetItemText(Index,2,text);

		switch((*itr).pricing_type)
		{
		case 1: sprintf_s(text, "SOV"); break;
		case 2: sprintf_s(text, "HOV"); break;
		case 3: sprintf_s(text, "Truck"); break;
		case 4: sprintf_s(text, "Intermodal"); break;
		default: sprintf_s(text, "---");
		}
		m_DemandTypeGrid.SetItemText(Index,3,text);

		sprintf_s(text, "%3.2f",(*itr).info_class_percentage[1]);
		m_DemandTypeGrid.SetItemText(Index,4,text);

		sprintf_s(text, "%3.2f",(*itr).info_class_percentage[2]);
		m_DemandTypeGrid.SetItemText(Index,5,text);

		for(int i=0; i< m_pDoc->m_VehicleTypeVector.size(); i++)
			{
		sprintf_s(text, "%3.2f",(*itr).vehicle_type_percentage[i]);
		m_DemandTypeGrid.SetItemText(Index,6+i,text);
			}
		
	}

	return TRUE;  // return TRUE  unless you set the focus to a control
}

BEGIN_MESSAGE_MAP(CDlg_DemandType, CDHtmlDialog)
	ON_BN_CLICKED(IDOK, &CDlg_DemandType::OnBnClickedOk)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_DemandTypeLIST, &CDlg_DemandType::OnLvnItemchangedDemandtypelist)
END_MESSAGE_MAP()

BEGIN_DHTML_EVENT_MAP(CDlg_DemandType)
	DHTML_EVENT_ONCLICK(_T("ButtonOK"), OnButtonOK)
	DHTML_EVENT_ONCLICK(_T("ButtonCancel"), OnButtonCancel)
END_DHTML_EVENT_MAP()



// CDlg_DemandType message handlers

HRESULT CDlg_DemandType::OnButtonOK(IHTMLElement* /*pElement*/)
{
	OnOK();
	return S_OK;
}

HRESULT CDlg_DemandType::OnButtonCancel(IHTMLElement* /*pElement*/)
{
	OnCancel();
	return S_OK;
}

void CDlg_DemandType::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
}

void CDlg_DemandType::OnLvnItemchangedDemandtypelist(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}
